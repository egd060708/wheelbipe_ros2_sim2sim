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

#ifndef TEMPLATE_ROS2_CONTROLLER__STATE_ERROR_HPP_
#define TEMPLATE_ROS2_CONTROLLER__STATE_ERROR_HPP_

#include "fsm/state_base.hpp"

namespace robot_locomotion
{

// 错误状态类
class StateError : public StateBase
{
public:
  StateError(StateMachine* state_machine, rclcpp::Logger logger);
  virtual ~StateError() = default;

  void enter(const RobotState& robot_state, const rclcpp::Time& time) override;
  void run(RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period) override;
  void exit(const RobotState& robot_state, const rclcpp::Time& time) override;
  std::string getName() const override { return "ERROR"; }
};

}  // namespace robot_locomotion

#endif  // TEMPLATE_ROS2_CONTROLLER__STATE_ERROR_HPP_

