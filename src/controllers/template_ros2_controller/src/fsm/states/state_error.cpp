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

#include "fsm/states/state_error.hpp"
#include "robot_state/robot_state.hpp"
#include "rclcpp/clock.hpp"

namespace robot_locomotion
{

StateError::StateError(StateMachine* state_machine, rclcpp::Logger logger)
  : StateBase(state_machine, logger)
{
}

void StateError::enter(const RobotState& robot_state, const rclcpp::Time& time)
{
  (void)robot_state;
  (void)time;
  RCLCPP_ERROR(logger_, "Entering ERROR state");
}

void StateError::run(RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period)
{
  (void)time;
  (void)period;
  // 错误状态：所有力矩设为0，确保安全
  for (auto& joint : robot_state.joints) {
    joint.output_torque = 0.0;
  }
  static rclcpp::Clock clock(RCL_ROS_TIME);
  RCLCPP_ERROR_THROTTLE(logger_, clock, 1000, "Controller in ERROR state");
}

void StateError::exit(const RobotState& robot_state, const rclcpp::Time& time)
{
  (void)robot_state;
  (void)time;
  RCLCPP_INFO(logger_, "Exiting ERROR state");
}

}  // namespace robot_locomotion

