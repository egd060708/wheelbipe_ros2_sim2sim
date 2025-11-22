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

#include "fsm/state_machine.hpp"
#include "robot_state/robot_state.hpp"
#include <algorithm>
#include "rclcpp/clock.hpp"

namespace robot_locomotion
{

void StateMachine::processEmergencyStopState(const RobotState& robot_state, const rclcpp::Time& time)
{
  (void)robot_state;
  (void)time;
  // 紧急停止状态：所有力矩立即设为0
  std::fill(output_torques_.begin(), output_torques_.end(), 0.0);
  static rclcpp::Clock clock(RCL_ROS_TIME);
  RCLCPP_WARN_THROTTLE(logger_, clock, 1000,
    "Controller in EMERGENCY_STOP state");
}

}  // namespace robot_locomotion

