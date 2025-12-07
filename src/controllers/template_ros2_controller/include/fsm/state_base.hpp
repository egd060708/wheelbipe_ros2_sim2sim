// Copyright (c) 2025 SCUT Robot Lab. All rights reserved.
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

#ifndef TEMPLATE_ROS2_CONTROLLER__STATE_BASE_HPP_
#define TEMPLATE_ROS2_CONTROLLER__STATE_BASE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot_locomotion
{

// 前向声明
struct RobotState;
class StateMachine;
class TensorRTInference;

// 状态基类
class StateBase
{
public:
  StateBase(StateMachine* state_machine, rclcpp::Logger logger);
  virtual ~StateBase() = default;

  // 进入状态时调用一次
  virtual void enter(const RobotState& robot_state, const rclcpp::Time& time) = 0;

  // 状态运行时循环调用
  virtual void run(RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period) = 0;

  // 退出状态时调用一次
  virtual void exit(const RobotState& robot_state, const rclcpp::Time& time) = 0;

  // 获取状态名称
  virtual std::string getName() const = 0;

protected:
  StateMachine* state_machine_;
  rclcpp::Logger logger_;
};

}  // namespace robot_locomotion

#endif  // TEMPLATE_ROS2_CONTROLLER__STATE_BASE_HPP_

