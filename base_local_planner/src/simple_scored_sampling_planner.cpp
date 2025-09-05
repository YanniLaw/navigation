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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/simple_scored_sampling_planner.h>

#include <ros/console.h>

namespace base_local_planner {
  
  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples) {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
  }

  // 解释下最后的这个得分: <0: 危险，应该丢弃该轨迹; >=0: 安全，得分越接近于0，轨迹就最好
  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost) {
    double traj_cost = 0;
    int gen_id = 0;
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {
        continue;
      }
      double cost = score_function_p->scoreTrajectory(traj); // 用不同的评价函数对轨迹进行打分
      if (cost < 0) { // 轨迹打分不满足要求，函数直接返回负数得分
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }
      traj_cost += cost;
      if (best_traj_cost > 0) {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        // 当前计算的得分比已经算出来的最好的得分还要大，那么就不再进行其他代价的打分啦！
        if (traj_cost > best_traj_cost) {
          break;
        }
      }
      gen_id ++;
    }


    return traj_cost;
  }

  bool SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) {
    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;
    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) { // 准备一些东西 主要是map grid
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }

    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) {
      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
      while (gen_->hasMoreTrajectories()) { // 其实就是判断是不是还可以生成打分的轨迹
        gen_success = gen_->nextTrajectory(loop_traj); // 生成待打分的轨迹
        if (gen_success == false) {
          // TODO use this for debugging
          continue;
        }
        loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost); // 对该轨迹进行打分
        if (all_explored != NULL) {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back(loop_traj);
        }

        if (loop_traj_cost >= 0) { // 当前轨迹安全
          count_valid++;
          // 首次进入该循环  或者 当前轨迹得分比已更新的best轨迹得分更好
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
            best_traj_cost = loop_traj_cost;
            best_traj = loop_traj;
          }
        }
        count++;
        if (max_samples_ > 0 && count >= max_samples_) { // 如果有设置最大轨迹数，那么就只处理这么多轨迹了
          break;
        }        
      }
      if (best_traj_cost >= 0) {
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;
        traj.cost_ = best_traj_cost;
        traj.resetPoints();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
          best_traj.getPoint(i, px, py, pth);
          traj.addPoint(px, py, pth);
        }
      }
      ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
      if (best_traj_cost >= 0) {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }

  
}// namespace
