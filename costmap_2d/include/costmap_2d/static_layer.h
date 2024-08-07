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
#ifndef COSTMAP_2D_STATIC_LAYER_H_
#define COSTMAP_2D_STATIC_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>

namespace costmap_2d
{

/*
* 插件类
* 继承自CostmapLayer类，实现了Layer类定义的虚函数。
* 继承于CostmapLayer,因为有自己的地图，Costmap2D提供了存储地图的父类，CostmapLayer提供了一些对地图的操作方法。
* 其初始化函数中订阅了map消息，所以必须要有mapserver提供map给它才行

* 关于该层数据的更新，如果不是滑动窗口类型,那么坐标框架就是和mapserver提供的map主题是一样的，只需要将订阅的数据copy过来更新自己的Costmap2D的数据成员就可以了， 
* 如果是滑动窗口,那么the master_grid is unlikely to have same coordinates as this layer， Copy map data given proper transformations。
* 即使是滑动窗口也是会利用全局图map提供的数据的。
*/
class StaticLayer : public CostmapLayer
{
public:
  StaticLayer();
  virtual ~StaticLayer();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();

private:
  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
  void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  unsigned char interpretValue(unsigned char value);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;  /// @brief frame that map is located in
  bool subscribe_to_updates_;
  bool map_received_; // 接收到了地图信息
  bool has_updated_data_;  // 更新数据
  unsigned int x_, y_, width_, height_; // 地图尺寸
  bool track_unknown_space_;
  bool use_maximum_;
  bool first_map_only_;      ///< @brief Store the first static map and reuse it on reinitializing
  bool trinary_costmap_;
  ros::Subscriber map_sub_, map_update_sub_;

  unsigned char lethal_threshold_, unknown_cost_value_;

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_STATIC_LAYER_H_
