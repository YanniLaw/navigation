/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <algorithm>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

InflationLayer::InflationLayer()
  : resolution_(0)
  , inflation_radius_(0)
  , inscribed_radius_(0)
  , weight_(0)
  , inflate_unknown_(false)
  , cell_inflation_radius_(0)
  , cached_cell_inflation_radius_(0)
  , dsrv_(NULL)
  , seen_(NULL)
  , cached_costs_(NULL)
  , cached_distances_(NULL)
  , last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
{
  inflation_access_ = new boost::recursive_mutex();
}

void InflationLayer::onInitialize()
{
  {
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    if (seen_)
      delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
    need_reinflation_ = false; // 整幅代价图是否要重新膨胀

    dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb =
        [this](auto& config, auto level){ reconfigureCB(config, level); };

    if (dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
  }

  matchSize();
}

void InflationLayer::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level)
{
  setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

  if (enabled_ != config.enabled || inflate_unknown_ != config.inflate_unknown) {
    enabled_ = config.enabled;
    inflate_unknown_ = config.inflate_unknown;
    need_reinflation_ = true;
  }
}

//根据master map的尺寸，更新本层的尺寸，将本层地图的大小设置为和LayeredCostmap::costmap_一样的大小
void InflationLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap(); // 主图层
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_); // inflation_radius_膨胀半径，膨胀层会把障碍物代价膨胀直到该半径为止
  computeCaches(); // 这个函数通过cell_inflation_radius_计算了两个buffer，这两个二维buffer直接存储了[i，j]的distance和cost

  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if (seen_)
    delete[] seen_;
  seen_size_ = size_x * size_y;
  seen_ = new bool[seen_size_];
}

// 确定最大的更新范围的边界 robot_x,robot_y,robot_yaw 是机器人的世界位姿
void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  // 由于膨胀层实际上没有维护地图数据，所以不存在真正意义上的地图边界
  if (need_reinflation_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max(); // 此时要对整幅代价图进行重新膨胀，所以不会理会以前调用updateBounds得到的min_x, min_y, max_x, max_y地图边界。没起到实质作用
    *max_y = std::numeric_limits<float>::max(); // 不过，在其他地方有判断，不论怎样都不会超过整幅地图的尺寸
    need_reinflation_ = false;
  }
  else
  {
    // 基本什么都没做，只是对前面调用updateBounds得到的min_x, min_y, max_x, max_y地图更新边界，进行膨胀
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // 由于LayeredCostmap::updateMap中取消了min_x, min_y, max_x, max_y的初始化，所以这里不能在min_x, min_y, max_x, max_y每次累加膨胀半径，
    // 而是新增了四个变量来记录加上膨胀半径的尺寸。
    // 实际上在该层的updateCosts中，已经加了一次膨胀半径，所以这里感觉有些多余，去掉之后目前测试未发现有何不同，代码先保留
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_; // 在传入的地图边界信息的基础上再扩大一个膨胀半径
  }
}

void InflationLayer::onFootprintChanged()
{
  inscribed_radius_ = layered_costmap_->getInscribedRadius(); // 内切半径
  cell_inflation_radius_ = cellDistance(inflation_radius_);   // 膨胀半径的栅格表示
  computeCaches(); // 更新两个维度为[cell_inflation_radius_+2][cell_inflation_radius_+2]的二维buffer：cached_costs_，cached_distances_
  need_reinflation_ = true; // 主要由于内切半径可能被更改了

  ROS_DEBUG("InflationLayer::onFootprintChanged(): num footprint points: %lu,"
            " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
            layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

// 对更新范围内的地图信息进行膨胀操作，即在LETHA_OBSTACLE像素的周围进行膨胀
void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  // 由于膨胀层实际上没有维护地图数据，只是对master图层的数据进行再次赋值而已
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  if (cell_inflation_radius_ == 0)
    return;

  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  ROS_ASSERT_MSG(inflation_cells_.empty(), "The inflation list must be empty at the beginning of inflation");

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (seen_ == NULL) {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  else if (seen_size_ != size_x * size_y)
  {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  memset(seen_, false, size_x * size_y * sizeof(bool));

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  //（min_i，min_j，max_i，max_j）是由上一层的地图已经在updateBounds中更新过，
  // InflationLayer层并未去改变它，这里将传入的boudns，按照机器人的膨胀尺寸，扩张这个bounds
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);

  // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

  // Start with lethal obstacles: by definition distance is 0.0
  std::vector<CellData>& obs_bin = inflation_cells_[0.0]; // 生成一个新的map项，key是0.0,以及一个空的vector实例
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE)
      {
        obs_bin.push_back(CellData(index, i, j, i, j)); // 存储master map中的obstacle cell信息，后面的(i,j)对是障碍物的索引，此时的距离为0
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  std::map<double, std::vector<CellData> >::iterator bin;
  // 逐次增加距离，新单元格被添加到相应的距离块中，std::map中有自动排序，这样它们就可以先处理，虽然存在以前插入的但是距离更远的
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
  {
    for (int i = 0; i < bin->second.size(); ++i) // 最初始检查的是distance为0.0的cell，也就是障碍物本身
    {
      // process all cells at distance dist_bin.first
      const CellData& cell = bin->second[i];

      unsigned int index = cell.index_;

      // ignore if already visited
      if (seen_[index])
      {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_; // sx,sy是障碍物的下标

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char cost = costLookup(mx, my, sx, sy);
      unsigned char old_cost = master_array[index];
      if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        master_array[index] = cost;
      else
        master_array[index] = std::max(old_cost, cost); // 有可能是相邻的障碍物单元格

      // attempt to put the neighbors of the current cell onto the inflation list，检查这个cell的前后左右四个cell
      //（sx, sy)没有被改变过，它一直是障碍物cell
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy); //左
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy); //上
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy); //右
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy);  //下
    }
  }

  inflation_cells_.clear();
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
// 新的cell将被添加到了map中相应的distance索引，由于map的key是到障碍物cell的距离，保证了循环能够按照距离的递增来处理cell
inline void InflationLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index]) // 避免cell被重复的塞进来
  {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y); // 当前cell（i, j）到障碍物cell的距离

    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_) // 保证了只处理在inflation_cell_dist_范围内的cell
      return;

    // push the cell data onto the inflation list and mark
    inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
  }
}

void InflationLayer::computeCaches()
{
  if (cell_inflation_radius_ == 0)
    return;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) // 膨胀尺寸发生改变
  {
    deleteKernels();

    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
      {
        cached_distances_[i][j] = hypot(i, j); // 栅格距离(hypot计算直角三角形的斜边长)
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
  {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
    {
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
    }
  }
}

void InflationLayer::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }

  if (cached_costs_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_costs_[i])
        delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
    cached_costs_ = NULL;
  }
}

void InflationLayer::setInflationParameters(double inflation_radius, double cost_scaling_factor)
{
  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
  {
    // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }
}

}  // namespace costmap_2d
