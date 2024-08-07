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
#ifndef COSTMAP_2D_INFLATION_LAYER_H_
#define COSTMAP_2D_INFLATION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

namespace costmap_2d
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;  // 在costmap的一维索引
  unsigned int x_, y_;  // 在costmap的二维索引
  unsigned int src_x_, src_y_;  // 距离当前栅格最近的障碍物在costmap的栅格索引
};

/*
* 插件类
* inflationLayer没有维护真正的地图数据所以只继承于Layer，该层所谓的地图层的概念，仅仅是操作master map的数据，Layer提供了操作master map的途径
*/
class InflationLayer : public Layer
{
public:
  InflationLayer();

  virtual ~InflationLayer()
  {
    deleteKernels();
    if (dsrv_)
        delete dsrv_;
    if (seen_)
        delete[] seen_;
  }

  virtual void onInitialize(); // overwrite function
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual bool isDiscretized()
  {
    return true;
  }
  virtual void matchSize();

  virtual void reset() { onInitialize(); }

  /** @brief  Given a distance, compute a cost.
   * @param  distance The distance from an obstacle in cells
   * @return A cost value for the distance */
  // 根据该点离其最近障碍物的距离来计算costmap损失值，是一个指数衰减关系。传入的distance表示栅格距离
  virtual inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0)
      cost = LETHAL_OBSTACLE; // 距离为0，则cost为LETHAL_OBSTACLE（254）
    else if (distance * resolution_ <= inscribed_radius_)
      cost = INSCRIBED_INFLATED_OBSTACLE;  // 距离小于机器人内切半径，则cost为INSCRIBED_INFLATED_OBSTACLE（253）
    else // 距离比内切半径远， 则距离障碍物越远cost取值越低
    {
      // make sure cost falls off by Euclidean distance
      double euclidean_distance = distance * resolution_;
      // 到实际障碍物的距离在内切圆半径到膨胀半径之间的所有cell可以使用如下公式来计算膨胀代价：
      // exp(-1.0 * cost_scaling_factor * (distance_from_obstacle - inscribed_radius)) * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1),
      // 由于在公式中cost_scaling_factor被乘了一个负数，所以增大比例因子反而会降低代价。
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  /**
   * @brief Change the values of the inflation radius parameters
   * @param inflation_radius The new inflation radius
   * @param cost_scaling_factor The new weight
   */
  void setInflationParameters(double inflation_radius, double cost_scaling_factor);

protected:
  virtual void onFootprintChanged();
  boost::recursive_mutex* inflation_access_;

  double resolution_;         // 分辨率
  double inflation_radius_;   // 膨胀半径(实际值)
  double inscribed_radius_;   // 内切半径
  double weight_;
  bool inflate_unknown_;

private:
  /**
   * @brief  Lookup pre-computed distances，查找预先计算的距离
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }

  /**
   * @brief  Lookup pre-computed costs
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_costs_[dx][dy];
  }

  void computeCaches();
  void deleteKernels();
  void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);

  // 将世界距离转换为栅格距离
  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
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
  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y);

  unsigned int cell_inflation_radius_;  // 栅格膨胀半径
  unsigned int cached_cell_inflation_radius_;   // 缓存栅格膨胀半径
  std::map<double, std::vector<CellData> > inflation_cells_;  // 膨胀栅格

  bool* seen_;
  int seen_size_; // 可视尺寸，其实就是栅格地图的一维大小

  unsigned char** cached_costs_;
  double** cached_distances_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_; // 上次地图边界

  dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig> *dsrv_;
  void reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level);

  bool need_reinflation_;  ///< Indicates that the entire costmap should be reinflated next time around.
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_INFLATION_LAYER_H_
