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

#include "fsm/state_base.hpp"
#include "fsm/state_machine.hpp"

namespace robot_locomotion
{

StateBase::StateBase(StateMachine* state_machine, rclcpp::Logger logger)
  : state_machine_(state_machine)
  , logger_(logger)
{
}

}  // namespace robot_locomotion

