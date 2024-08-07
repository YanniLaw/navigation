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
#ifndef COSTMAP_2D_LAYERED_COSTMAP_H_
#define COSTMAP_2D_LAYERED_COSTMAP_H_

#include <costmap_2d/cost_values.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <string>

namespace costmap_2d
{
class Layer;

/**
 * @class LayeredCostmap "包工头"
 * @brief Instantiates different layer plugins and aggregates them into one score
 * 图层管理类，管理各个图层插件，并将信息汇总到主图层master costmap上， 并提供给Costmap2DROS调用接口。
 */
class LayeredCostmap
{
public:
  /**
   * @brief  Constructor for a costmap
   */
  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

  /**
   * @brief  Destructor
   */
  ~LayeredCostmap();

  /**
   * @brief  Update the underlying costmap with new data.用新数据更新底层costmap
   * If you want to update the map outside of the update loop that runs, you can call this.
   */
  void updateMap(double robot_x, double robot_y, double robot_yaw);

  inline const std::string& getGlobalFrameID() const noexcept
  {
    return global_frame_;
  }

  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                 bool size_locked = false);

  void getUpdatedBounds(double& minx, double& miny, double& maxx, double& maxy)
  {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  bool isCurrent();

  Costmap2D* getCostmap() // 返回主图层
  {
    return &costmap_;
  }

  bool isRolling()
  {
    return rolling_window_;
  }

  bool isTrackingUnknown()
  {
    return costmap_.getDefaultValue() == costmap_2d::NO_INFORMATION;
  }

  std::vector<boost::shared_ptr<Layer> >* getPlugins()
  {
    return &plugins_;
  }

  void addPlugin(boost::shared_ptr<Layer> plugin)
  {
    plugins_.push_back(plugin);
  }

  bool isSizeLocked()
  {
    return size_locked_;
  }

  void getBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }

  bool isInitialized()
  {
      return initialized_;
  }

  /** @brief Updates the stored footprint, updates the circumscribed
   * and inscribed radii, and calls onFootprintChanged() in all
   * layers. */
  void setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec);

  /** @brief Returns the latest footprint stored with setFootprint(). */
  const std::vector<geometry_msgs::Point>& getFootprint() { return footprint_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which just surrounds all points on the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getCircumscribedRadius() { return circumscribed_radius_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which is just within all points and edges of the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getInscribedRadius() { return inscribed_radius_; }

private:
  Costmap2D costmap_;  // 主图层 master costmap，各个图层的数据叠加之后的代价地图对象
  std::string global_frame_;  // 全局坐标系

  bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot，局部costmap设置为true

  bool current_; // 判断地图是否有效
  double minx_, miny_, maxx_, maxy_;
  unsigned int bx0_, bxn_, by0_, byn_; // 地图边界

  std::vector<boost::shared_ptr<Layer> > plugins_; // 图层插件,记录了所有的图层

  bool initialized_;  // 其实是成功更新地图的标志
  bool size_locked_;
  // inscribed_radius_, circumscribed_radius_ 是计算得到的机器人底座的内切圆和外接圆半径
  double circumscribed_radius_, inscribed_radius_;
  std::vector<geometry_msgs::Point> footprint_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_LAYERED_COSTMAP_H_
