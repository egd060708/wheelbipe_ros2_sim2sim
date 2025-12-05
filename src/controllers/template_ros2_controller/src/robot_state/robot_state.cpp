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

#include "robot_state/robot_state.hpp"
#include <algorithm>
#include "utils/orientation_tools.h"

namespace robot_locomotion
{

void RobotState::run()
{
  this->body_state.ang_vel_b = Eigen::Map<const Vec3<double>>(imu.angular_velocity.data());
  this->body_state.lin_acc_b = Eigen::Map<const Vec3<double>>(imu.linear_acceleration.data());
  this->body_state.orientation_b = Eigen::Map<const Quat<double>>(imu.orientation.data());
  // 计算旋转矩阵
  this->body_state.rotation_w2b = ori::quaternionToRotationMatrix(this->body_state.orientation_b);
  this->body_state.rotation_b2w = this->body_state.rotation_w2b.transpose();
  // 计算滚转俯仰偏航角
  this->body_state.rpy = ori::quatToRPY(this->body_state.orientation_b);
}

JointState* RobotState::findJoint(const std::string& joint_name)
{
  auto it = std::find_if(joints.begin(), joints.end(),
    [&joint_name](const JointState& joint) {
      return joint.name == joint_name;
    });
  return (it != joints.end()) ? &(*it) : nullptr;
}

const JointState* RobotState::findJoint(const std::string& joint_name) const
{
  auto it = std::find_if(joints.begin(), joints.end(),
    [&joint_name](const JointState& joint) {
      return joint.name == joint_name;
    });
  return (it != joints.end()) ? &(*it) : nullptr;
}

std::vector<double> RobotState::getOutputTorques() const
{
  std::vector<double> torques;
  torques.reserve(joints.size());
  for (const auto& joint : joints) {
    torques.push_back(joint.output_torque);
  }
  return torques;
}

std::vector<double> RobotState::getPositions() const
{
  std::vector<double> positions;
  positions.reserve(joints.size());
  for (const auto& joint : joints) {
    positions.push_back(joint.position);
  }
  return positions;
}

std::vector<double> RobotState::getVelocities() const
{
  std::vector<double> velocities;
  velocities.reserve(joints.size());
  for (const auto& joint : joints) {
    velocities.push_back(joint.velocity);
  }
  return velocities;
}

std::vector<double> RobotState::getEfforts() const
{
  std::vector<double> efforts;
  efforts.reserve(joints.size());
  for (const auto& joint : joints) {
    efforts.push_back(joint.effort);
  }
  return efforts;
}

}  // namespace robot_locomotion

