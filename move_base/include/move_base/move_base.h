/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum MoveBaseState {
    PLANNING,     // 全局路径规划
    CONTROLLING,  // 局部路径规划
    CLEARING      // 恢复状态，清图
  };

  enum RecoveryTrigger
  {
    PLANNING_R,     // 全局路径规划触发了清图 
    CONTROLLING_R,  // 局部路径规划触发了清图
    OSCILLATION_R   // 局部路径规划中由于振荡触发了清图
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf2_ros::Buffer& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf2_ros::Buffer& tf_;

      MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_; // 局部规划器(trajectory controller)
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_; // 全局规划器及局部规划器的代价地图对象

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_; // 全局规划器
      std::string robot_base_frame_, global_frame_; // global_costmap下的机器人坐标系，全局坐标系

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      std::vector<std::string> recovery_behavior_names_;
      unsigned int recovery_index_;

      geometry_msgs::PoseStamped global_pose_;
	  // 全局规划器的执行频率，如果为0则只有出现新的目标点或者局部规划器报告路径堵塞时，才会重新规划
	  // 向底盘控制移动话题cmd_vel发送速度命令的频率,这个速度由base_local_planner计算
	  // 内切半径
	  // 外切半径
	  // 进行全局规划的时间间隔，如果超时则认为规划失败; 在空间清理操作执行前,留给规划器多长时间来找出一条有效规划
	  // 等待控制速度的时间间隔，如果控制速度的发布超过设置时间，则认为局部路径规划失败
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_; // 在执行恢复行为之前允许计划重试的次数.默认为-1，表示全局规划失败后立即执行恢复模块
      uint32_t planning_retries_; // 重新尝试进行全局路径规划的次数
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;
      ros::Subscriber goal_sub_;
	  // 只会提供plan该怎么走的位置信息，不会使机器人移动
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_; // 告诉move_base清除costmap中的障碍物信息，可能导致撞到障碍物
	  // 当move_base不活动时，是否关闭代价地图的加载
	  // 是否允许旋转恢复行为
	  // 是否使用恢复模块
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
	  // 在执行恢复行为之前允许振荡的时间（秒
	  //机器人必须移动多远（以米计）才能被视为不摆动。如果出现摆动则说明全局规划失败，那么将在超时后执行恢复模块。
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

	  // 上一次有效全局路径规划的时间
	  // 上一次有效控制的时间
	  // 上一次振荡重置的时间
      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_; // 振荡位姿
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer, 路径规划结果缓存
      std::vector<geometry_msgs::PoseStamped>* planner_plan_; // 先保存全局规划器最新规划出来的的路径，然后赋值给latest_plan_
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;  // 保存全局规划器最新规划出来的全局路径
      std::vector<geometry_msgs::PoseStamped>* controller_plan_; // 由latest_plan_赋值而得，局部规划器执行的路径(全局)

      //set up the planner's thread
      bool runPlanner_; // 是否运行全局规划线程
      boost::recursive_mutex planner_mutex_;
      boost::condition_variable_any planner_cond_; // 全局规划线程
      geometry_msgs::PoseStamped planner_goal_; // 目标地点
      boost::thread* planner_thread_; // 全局路径规划线程


      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
	  // 动态参数配置时控制频率发生了变化
      bool setup_, p_freq_change_, c_freq_change_; // 新规划出来了全局路径
      bool new_global_plan_;
  };
};
#endif

