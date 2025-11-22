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

#ifndef TEMPLATE_ROS2_CONTROLLER__STATE_MACHINE_HPP_
#define TEMPLATE_ROS2_CONTROLLER__STATE_MACHINE_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace robot_locomotion
{

// 前向声明
struct RobotState;

// 状态机状态枚举
enum class ControllerState
{
  INIT = 0,           // 初始化状态
  IDLE,               // 空闲状态
  STANDING,           // 站立状态
  WALKING,            // 行走状态
  RUNNING,            // 跑步状态
  ERROR,              // 错误状态
  EMERGENCY_STOP      // 紧急停止状态
};

// 状态机类
class StateMachine
{
public:
  StateMachine(rclcpp::Logger logger);
  ~StateMachine() = default;

  // 状态机更新函数，根据当前状态和机器人状态进行状态转换和输出计算
  void update(const RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period);

  // 获取当前状态
  ControllerState getCurrentState() const { return current_state_; }

  // 获取状态名称
  std::string getStateName() const;

  // 设置目标状态（用于外部控制）
  void setTargetState(ControllerState target_state);

  // 获取输出力矩（由状态机计算）
  const std::vector<double>& getOutputTorques() const { return output_torques_; }

  // 重置状态机
  void reset();

protected:
  // 状态转换逻辑
  void handleStateTransition(const RobotState& robot_state, const rclcpp::Time& time);

  // 各状态的处理函数（由各个状态文件实现）
  void processInitState(const RobotState& robot_state, const rclcpp::Time& time);
  void processIdleState(const RobotState& robot_state, const rclcpp::Time& time);
  void processStandingState(const RobotState& robot_state, const rclcpp::Time& time);
  void processWalkingState(const RobotState& robot_state, const rclcpp::Time& time);
  void processRunningState(const RobotState& robot_state, const rclcpp::Time& time);
  void processErrorState(const RobotState& robot_state, const rclcpp::Time& time);
  void processEmergencyStopState(const RobotState& robot_state, const rclcpp::Time& time);

  // 计算输出力矩
  void computeOutputTorques(const RobotState& robot_state);

  ControllerState current_state_;
  ControllerState target_state_;
  rclcpp::Logger logger_;
  std::vector<double> output_torques_;
  rclcpp::Time state_entry_time_;
  size_t num_joints_;
};

}  // namespace robot_locomotion

#endif  // TEMPLATE_ROS2_CONTROLLER__STATE_MACHINE_HPP_

