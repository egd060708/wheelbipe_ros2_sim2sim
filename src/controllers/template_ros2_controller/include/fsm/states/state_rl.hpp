// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEMPLATE_ROS2_CONTROLLER__STATE_RL_HPP_
#define TEMPLATE_ROS2_CONTROLLER__STATE_RL_HPP_

#include "fsm/state_base.hpp"
#include <thread>
#include <functional>
#include <mutex>
#include "controller_modules/PIDmethod.h"
#include "robot_state/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl/time.h"

namespace robot_locomotion
{

struct ModelParams
{
    float joint_damping;
    float joint_stiffness;
    float wheel_damping;
    float wheel_stiffness;
    float joint_action_scale;
    float wheel_action_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float cmd_scale;
    float height_scale;
    float default_dof_pos[6] = {0};
    float spring_force;
};

// // 观测数据维度
// struct Observations
// {          
//     float ang_vel[3];  
//     float gravity_vec[3];
//     float dof_pos[6];         
//     float dof_vel[6];  
//     float commands[3];        
//     float height_cmd;
//     float last_actions[6];
// };

enum DofCtrlType
{
  P=0,
  V,
  T
};
// // 电机执行维度
// struct DofCtrl
// {
//   DofCtrlType dof_mode = P;
//   float P_p[6];
//   float P_d[6];
//   float V_p[6];
//   float V_d[6];
//   float T_d[6];
// };

// 强化学习状态类
class StateRL : public StateBase
{
public:
  StateRL(StateMachine* state_machine, rclcpp::Logger logger);
  virtual ~StateRL() = default;

  void enter(const RobotState& robot_state, const rclcpp::Time& time) override;
  void run(RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period) override;
  void exit(const RobotState& robot_state, const rclcpp::Time& time) override;
  std::string getName() const override { return "RL"; }
  
  // 设置调度模式：0=线程(系统时间)，1=ROS2定时器，2=内联（run 内不执行底层）
  void setMode(int mode, rcl_clock_type_t clock_type, double freq_hz = 0.0,
               rclcpp_lifecycle::LifecycleNode::SharedPtr node = nullptr);
  
  // ROS2 定时器回调函数（模式1：定时器调度）
  void onLowlevelTimer();

  // 简化底层策略
  void simplelowlevelCallbask(RobotState& robot_state, const rclcpp::Time& time);

private:
  std::array<float, 6> last_actions_ = {0};
  
  ModelParams params_;
  PIDmethod pid_controllers[6];
  void lowlevelThreadLoop();  // 线程循环函数（模式0：独立线程调度）
  void doLowlevelStep();  // 公共的低层控制执行逻辑
  DofCtrlType dof_mode = P;
  using Dof_Actuate = std::function<void()>;  // 改用 std::function
  Dof_Actuate actuate[3];  // 存储可调用对象
  void _P_actuate();
  void _V_actuate();
  void _T_actuate();
  std::thread actuate_thread_;
  bool threadRunning = false;
  bool stop_update_ = false;
  bool thread_first_ = true;

  double desired_pos[6] = {0};
  double torque[6] = {0};

  // 线程安全的 robot_state 副本（用于低层控制线程）
  RobotState robot_state_copy_;
  std::mutex robot_state_mutex_;
  
  // 用于记录上次执行时间（微秒）
  long long last_execution_time_ = 0;
  
  // 调度模式与时钟
  int mode_ = 0;
  rcl_clock_type_t timer_clock_type_ = RCL_ROS_TIME;
  
  // ROS2 定时器（当使用 ROS2 时间/其他时钟时）
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr ros_timer_;
  long long period_microseconds_ = 0;
  long long lowlevel_period_us_ = 2000;  // 默认 500Hz
};

}  // namespace robot_locomotion

#endif  // TEMPLATE_ROS2_CONTROLLER__STATE_RL_HPP_

