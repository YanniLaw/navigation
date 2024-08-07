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
#ifndef COSTMAP_2D_OBSTACLE_LAYER_H_
#define COSTMAP_2D_OBSTACLE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObstaclePluginConfig.h>
#include <costmap_2d/footprint.h>

namespace costmap_2d
{

/*
* 插件类
* 继承自CostmapLayer类，实现了Layer类定义的虚函数。
* 继承于CostmapLayer,因为有自己的地图，Costmap2D提供了存储地图的父类，CostmapLayer提供了一些对地图的操作方法。
*/
class ObstacleLayer : public CostmapLayer
{
public:
  ObstacleLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual ~ObstacleLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  /**
   * @brief  A callback to handle buffering LaserScan messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                         const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

   /**
    * @brief A callback to handle buffering LaserScan messages which need filtering to turn Inf values into range_max.
    * @param message The message returned from a message notifier
    * @param buffer A pointer to the observation buffer to update
    */
  void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& message,
                                 const boost::shared_ptr<ObservationBuffer>& buffer);

  /**
   * @brief  A callback to handle buffering PointCloud messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                          const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

  /**
   * @brief  A callback to handle buffering PointCloud2 messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                           const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

  // for testing purposes
  void addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing);
  void clearStaticObservations(bool marking, bool clearing);

protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  /**
   * @brief  Get the observations used to mark space
   * @param marking_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;

  /**
   * @brief  Get the observations used to clear space
   * @param clearing_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getClearingObservations(std::vector<costmap_2d::Observation>& clearing_observations) const;

  /**
   * @brief  Clear freespace based on one observation
   * @param clearing_observation The observation used to raytrace
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                 double* max_x, double* max_y);

  void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  std::vector<geometry_msgs::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, 
                       double* max_x, double* max_y);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  double max_obstacle_height_;  ///< @brief Max Obstacle Height

  laser_geometry::LaserProjection projector_;  ///< @brief Used to project laser scans into point clouds

  // 可能有几个扫描数据的来源，所以用到vector，里面的每一个元素对应一个观察源，用于存放观察源的相关数据
  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
  std::vector<boost::shared_ptr<tf2_ros::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
  // 三个缓冲区数组，用于存放三类缓冲区：观察缓冲区、mark缓冲区、clear缓冲区
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > marking_buffers_;  ///< @brief Used to store observation buffers used for marking obstacles
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > clearing_buffers_;  ///< @brief Used to store observation buffers used for clearing obstacles

  // Used only for testing purposes
  std::vector<costmap_2d::Observation> static_clearing_observations_, static_marking_observations_;

  bool rolling_window_; // 是否是滑动窗口
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig> *dsrv_;

  int combination_method_;

private:
  void reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level);
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_OBSTACLE_LAYER_H_
