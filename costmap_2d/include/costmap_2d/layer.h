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
 * Author: David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_LAYER_H_
#define COSTMAP_2D_LAYER_H_

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <string>
#include <tf2_ros/buffer.h>

namespace costmap_2d
{
class LayeredCostmap;

class Layer
{
public:
  Layer();

  void initialize(LayeredCostmap* parent, std::string name, tf2_ros::Buffer *tf);

  /**
   * @brief This is called by the LayeredCostmap to poll this plugin as to how
   *        much of the costmap it needs to update. Each layer can increase
   *        the size of this bounds.
   * LayeredCostmap调用这个函数来询问这个插件需要更新多少costmap。每一层都可以增加这个边界的大小。
   * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
   * by Lu et. Al, IROS 2014.
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y) {}

  /**
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds(). 更新底层costmap
   */
  virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {}

  /** @brief Stop publishers. 禁用*/
  virtual void deactivate() {}

  /** @brief Restart publishers if they've been stopped. 激活 */
  virtual void activate() {}

  virtual void reset() {}

  virtual ~Layer() {}

  /**
   * @brief Check to make sure all the data in the layer is up to date.
   *        If the layer is not up to date, then it may be unsafe to
   *        plan using the data from this layer, and the planner may
   *        need to know.
   *
   *        A layer's current state should be managed by the protected
   *        variable current_.
   * @return Whether the data in the layer is up to date. 检查以确保Layer中的所有数据都是最新的。
   */
  bool isCurrent() const
  {
    return current_;
  }

  /**
   * @brief getter if the current layer is enabled.
   *
   * The user may enable/disable a layer though dynamic reconfigure.
   * Disabled layers won't receive calls to
   * - Layer::updateCosts
   * - Layer::updateBounds
   * - Layer::isCurrent
   * from the LayeredCostmap.
   *
   * Calls to Layer::activate, Layer::deactivate and Layer::reset won't be
   * blocked.
   */
  inline bool isEnabled() const noexcept
  {
    return enabled_;
  }

  /** @brief Implement this to make this layer match the size of the parent costmap. */
  virtual void matchSize() {}

  inline const std::string& getName() const noexcept
  {
    return name_;
  }

  /** @brief Convenience function for layered_costmap_->getFootprint(). */
  const std::vector<geometry_msgs::Point>& getFootprint() const;

  /** @brief LayeredCostmap calls this whenever the footprint there
   * changes (via LayeredCostmap::setFootprint()).  Override to be
   * notified of changes to the robot's footprint. */
  virtual void onFootprintChanged() {}

protected:
  /** @brief This is called at the end of initialize().  Override to
   * implement subclass-specific initialization.
   *
   * tf_, name_, and layered_costmap_ will all be set already when this is called. */
  virtual void onInitialize() {}

  LayeredCostmap* layered_costmap_; // 通过该指针获取到对主图层master map的操作。没有这个指针，所有基于Layer继承下去的地图的类，都无法操作master map
  bool current_; // layer的数据是否都是最新的
  bool enabled_;  ///< Currently this var is managed by subclasses. TODO: make this managed by this class and/or container class.
  std::string name_;
  tf2_ros::Buffer *tf_;

private:
  std::vector<geometry_msgs::Point> footprint_spec_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_LAYER_H_
