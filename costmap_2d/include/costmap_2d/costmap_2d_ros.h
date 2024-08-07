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
#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.hpp>
#include <tf2/LinearMath/Transform.h>

class SuperValue : public XmlRpc::XmlRpcValue
{
public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
  {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

/*
* 对于costmap来说，最内部的数据类型为Costmap2D，其内部通过unsigned char* costmap_来保存最基本的地图数据。
* 总体层次结构为：Costmap2DROS内部保存的地图相关对象为LayeredCostmap，而LayeredCostmap内部有一个Costmap2D用来保存总的地图信息，
* 另外它还包括了层信息std::vector<boost::shared_ptr<Layer>> plugins（层的类型可以是StaticLayer、ObstacleLayer、InflationLayer）。
* LayeredCostmap最终会将各层的信息汇总到其内部的Costmap2D中。
* LayeredCostmap中的Costmap2D在初始化构造时采用了默认构造函数，将地图尺寸、分辨率、原点信息、地图数据都设置为0或者NULL。
* 然后，LayeredCostmap根据传入进来的在Costmap2DROS中捕获到track_unkown_space参数先给其内部的总地图Costmap2D设置地图数据缺省值NO_INFORMATION或者FREE_SPACE。

* StaticLayer继承自Layer和Costmap2D（其它两层相同），因此相当于通过Costmap2D保存地图信息。StaticLayer在构造时调用默认构造函数，
* 因此Costmap2D也通过默认构造函数进行了设置。在静态层的onInitialize函数中，获取到了在该层作用域中的track_unkown_space和unknown_cost_value两个参数，
* 这两个参数不是用来设置Costmap2D中地图数据的缺省值的，而是用来在接收到地图数据后解析地图中的未知区域信息（值等于unknown_cost_value）的，
* 根据track_unknown_space的值不同，将订阅到的地图信息中的unknown_cost_value转化为NO_INFORMATION或者FREE_SPACE。
* 由于静态层的数据会完全被订阅的地图信息赋值，因此缺省值对其来说不重要（保持默认构造为0）。

* ObtacleLayer的构造及初始化方式和StaticLayer类似。在该层的OnInitialize函数中，获取到了在该层作用域中的track_unknown_space参数，
* 然后根据该参数对其继承自Costmap2D的default_value_参数赋值为NO_INFORMATION或者FREE_SPACE。

* InflationLayer的构造及初始化方法和StaticLayer类似。该层是基于已有地图进行膨胀，因此不需要有缺省值。

* 地图各层会有自己的更新机制，例如静态地图根据map话题更新自己的数据（也可以设置成只更新一次），动态地图则根据激光数据更新地图，膨胀层不会更新地图，只是对已有地图进行膨胀。
* 在总地图（即Costmap2DROS->LayeredCostmap->Costmap2D）更新时，是将重置过的总地图依次传入各层，利用各层的数据进行更新。
* 默认情况下，静态层会用自己的信息覆盖传入的总地图信息；障碍物层会用自己的信息和传入的总地图信息进行比对，然后取最大值（NO_INFORMATION虽然为255，但不参与比较直接略过）;
* 膨胀层则对传入的总地图信息执行膨胀操作。因此各层的放置顺序不能随意颠倒。
*/

namespace costmap_2d
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */
// 从ROS用户的角度，只需要调用Costmap2DROS这个类，因为这个类已经把所有关于地图的操作都封装好了。
// Costmap2DROS是提供给用户的主要接口, 是对Costmap2D的封装, 可以在初始化时指定其命名空间。真正的地图信息是储存在各个Layer中
class Costmap2DROS
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS(const std::string &name, tf2_ros::Buffer& tf);
  ~Costmap2DROS();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  void stop();

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   * 暂停更新代价地图，不操作图层对象
   */
  void pause();

  /**
   * @brief  Resumes costmap updates 恢复更新代价地图，不操作图层对象
   */
  void resume();

  void updateMap();

  /**
   * @brief Reset each individual layer
   */
  void resetLayers();

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  //判断地图是否有效
  /*
  * 对于各层，都有current_参数，该参数继承自Layer类，且通过Layer::isCurrent查询。总的地图的current是通过各层current的与操作计算出来的。
  * 对于静态层，只要初始化（onInitialize）完成后current_就一直为true
  * 对于障碍物层，其current_参数是各个观察缓冲区（ObservationBuffer）是否current的与操作，
  * 而ObservationBuffer的current取决于缓冲区更新时间（新的观测数据到来的时间）与expected_update_rate参数的对比。
  * 对于膨胀层，也是只要初始化完成后curretn_就一直为true
  */
  bool isCurrent() const
    {
      return layered_costmap_->isCurrent();
    }

  /**
   * @brief Is the costmap stopped
   */
  bool isStopped() const
    {
      return stopped_;
    }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  bool getRobotPose(geometry_msgs::PoseStamped& global_pose) const;

  /** @brief Returns costmap name */
  inline const std::string& getName() const noexcept
    {
      return name_;
    }

  /** @brief Returns the delay in transform (tf) data that is tolerable in seconds */
  double getTransformTolerance() const
    {
      return transform_tolerance_;
    }

  /** @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   * 获取主图层
   * Same as calling getLayeredCostmap()->getCostmap(). */
  Costmap2D* getCostmap() const
    {
      return layered_costmap_->getCostmap();
    }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  inline const std::string& getGlobalFrameID() const noexcept
    {
      return global_frame_;
    }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  inline const std::string& getBaseFrameID() const noexcept
    {
      return robot_base_frame_;
    }
  LayeredCostmap* getLayeredCostmap() const
    {
      return layered_costmap_;
    }

  /** @brief Returns the current padded footprint as a geometry_msgs::Polygon. */
  geometry_msgs::Polygon getRobotFootprintPolygon() const
  {
    return costmap_2d::toPolygon(padded_footprint_);
  }

  /** @brief Return the current footprint of the robot as a vector of points.
   * 在机器人坐标系下
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  inline const std::vector<geometry_msgs::Point>& getRobotFootprint() const noexcept
  {
    return padded_footprint_;
  }

  /** @brief Return the current unpadded footprint of the robot as a vector of points.
   *  在机器人坐标系下
   * This is the raw version of the footprint without padding.
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  inline const std::vector<geometry_msgs::Point>& getUnpaddedRobotFootprint() const noexcept
  {
    return unpadded_footprint_;
  }

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose（在地图全局坐标系下，包含padding）
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;

  /** @brief Set the footprint of the robot to be the given set of
   * points, padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points);

  /** @brief Set the footprint of the robot to be the given polygon,
   * padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint);

protected:
  //使用LayeredCostmap来跟踪每一层, 每一层以插件方式被实例化，并被添加到LayeredCostmap，每个插件（即地图层）都是Layer类型
  LayeredCostmap* layered_costmap_; // 图层管理器，记录了代价地图的各个图层对象，并提供融合各个图层数据的接口
  std::string name_; // costmap对象名称,是全局costmap还是局部costmap
  tf2_ros::Buffer& tf_;  ///< @brief Used for transforming point clouds
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_;  ///< timeout before transform errors

private:
  /** @brief Set the footprint from the new_config object.
   *
   * If the values of footprint and robot_radius are the same in
   * new_config and old_config, nothing is changed. */
  void readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                               const costmap_2d::Costmap2DConfig &old_config);

  void loadOldParameters(ros::NodeHandle& nh);
  void warnForOldParameters(ros::NodeHandle& nh);
  void checkOldParam(ros::NodeHandle& nh, const std::string &param_name);
  void copyParentParameters(const std::string& plugin_name, const std::string& plugin_type, ros::NodeHandle& nh);
  void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);
  void movementCB(const ros::TimerEvent &event);
  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_; // 是否终止更新地图的线程
  // 用于标记是否停止更新代价地图，与图层的工作状态无关
  // 用于标记代价地图是否已经初始化，更新地图线程是否正常工作
  // 用于标记是否关闭了各个图层
  // 机器人是否停止运动
  bool stop_updates_, initialized_, stopped_;
  boost::thread* map_update_thread_;  ///< @brief A thread for updating the map 用于更新代价地图的线程
  ros::Time last_publish_; // 上次发布costmap的时间
  ros::Duration publish_cycle; // 发布代价地图的周期计数器
  pluginlib::ClassLoader<Layer> plugin_loader_;
  Costmap2DPublisher* publisher_;
  dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;

  boost::recursive_mutex configuration_mutex_;

  ros::Subscriber footprint_sub_;
  ros::Publisher footprint_pub_; // 用于发布机器人足迹(轮廓)
  std::vector<geometry_msgs::Point> unpadded_footprint_;  // 未填充的足迹
  std::vector<geometry_msgs::Point> padded_footprint_;    // 填充后的足迹
  float footprint_padding_; // 足迹填充大小
  costmap_2d::Costmap2DConfig old_config_;
};
// class Costmap2DROS
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_ROS_H
