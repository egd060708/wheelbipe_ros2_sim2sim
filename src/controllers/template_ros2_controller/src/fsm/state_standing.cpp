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

namespace robot_locomotion
{

void StateMachine::processStandingState(RobotState& robot_state, const rclcpp::Time& time)
{
  (void)time;
  // 站立状态：保持平衡，可以添加平衡控制逻辑
  // 这里先设置为0，后续可以根据需要添加控制算法
  for (auto& joint : robot_state.joints) {
    joint.output_torque = 0.0;
  }
}

}  // namespace robot_locomotion

