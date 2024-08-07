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
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using std::vector;

namespace costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown) :
    costmap_(),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    current_(false),
    minx_(0.0),
    miny_(0.0),
    maxx_(0.0),
    maxy_(0.0),
    bx0_(0),
    bxn_(0),
    by0_(0),
    byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.1)
{
  if (track_unknown)
    costmap_.setDefaultValue(NO_INFORMATION);
  else
    costmap_.setDefaultValue(FREE_SPACE);
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}

void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  // 全局代价地图时，master map的尺寸始终和静态地图的尺寸一样。在局部代价地图动态窗口时，master map的尺寸由外部配置文件设定 
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  size_locked_ = size_locked;
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->matchSize(); // 调整各个图层的地图尺寸，实际效果就是将plugin所指向的每一层地图的大小都设置为和LayeredCostmap::costmap_一样的空间大小
  }
}

//传入的robot_x，robot_y，robot_yaw都是在全局坐标系下的值
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

  // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
  if (rolling_window_) // 适用于局部costmap的情况
  {
    // 简而言之，就是始终以当前机器人位置为局部costmap最中心，机器人移动多少，新的原点就相对于上一次的地图原点移动多少
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2; // 这幅地图是以机器人为中心
    costmap_.updateOrigin(new_origin_x, new_origin_y); //变换地图的起始点坐标，就表示机器人移动了
  }

  if (plugins_.size() == 0)
    return;
  /*****
   * 这里是原生的两行代码，现在注释掉，相关解释如下：
   * 1.静态层监听的是gmapping发来的map消息，全局图的尺寸是640*640，所以每次在静态层updateBounds之后，minx_，miny_，maxx_，maxy_这四个值即为整个全局图尺寸坐标，
   *   后续层的updateBounds和updateCosts也会在这四个值范围内完成
   * 2.由于现在gmapping功能和障碍物层雷同了，所以去掉了静态层不再接收gmapping的消息
   * 3.在停用了静态层后，保险杠层和障碍物层的updateBounds只能实时更新探测范围内的边界（根据点云），所以这四个值即只指示了机器人周边一部分区域，
   *   所以导致在每次做膨胀操作时，只能对机器人周边的部分区域做膨胀，这是不合理的（我们希望对所有已知障碍物膨胀），但是如果在膨胀层将膨胀尺寸设为全局图的固定尺寸640*640,又有点浪费，
   *   我们只需要对已探测到的最大区域做膨胀
   * 4.注释掉这里四个值的初始化，用它们来指示已探测到的最大区域，这样在每次导航之前，对最大区域做膨胀，导航完成后，障碍物层会对最大区域做复原（updateCosts采用了overwrite方式）
  *****/
  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;

  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if(!(*plugin)->isEnabled())
      continue;
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    // 根据各层的更新情况，确定最大的更新范围的边界
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
    }
  }

  int x0, xn, y0, yn; // 边界
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);

  ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0)
    return;

  // 重置地图并重新计算各个栅格的代价，并标识地图更新状态
  costmap_.resetMap(x0, y0, xn, yn); // 用缺省值重置代价地图更新边界范围内的地图信息(主图层)
  // 地图信息的更新并不是每次都更新整幅地图，而是根据计算出的更新边界，然后用各层去更新该边界内的区域
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if((*plugin)->isEnabled())
      (*plugin)->updateCosts(costmap_, x0, y0, xn, yn); // 用各层的信息在更新边界内部更新master map地图信息，这个阶段将逐一拷贝数据到Master Map
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

// 对操作的实时性提供保证，提供是否发生超时的信息
bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    if((*plugin)->isEnabled())
      current_ = current_ && (*plugin)->isCurrent(); // layer是否是最新的数据
  }
  return current_;
}

// 设置新的足迹模型
void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  // inscribed_radius_, circumscribed_radius_ 是计算得到的机器人底座的内切圆和外接圆半径
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged(); // 主要是针对膨胀层
  }
}

}  // namespace costmap_2d
