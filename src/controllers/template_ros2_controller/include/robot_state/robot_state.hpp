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

#ifndef TEMPLATE_ROS2_CONTROLLER__ROBOT_STATE_HPP_
#define TEMPLATE_ROS2_CONTROLLER__ROBOT_STATE_HPP_

#include <vector>
#include <string>
#include <array>
#include "utils/cppTypes.h"

namespace robot_locomotion
{

// 关节状态结构
struct JointState
{
  std::string name;           // 关节名称
  double position = 0.0;      // 关节位置 (rad)
  double velocity = 0.0;      // 关节速度 (rad/s)
  double effort = 0.0;        // 关节力矩 (N·m)
  double output_torque = 0.0; // 输出力矩 (N·m)
};

// IMU传感器状态结构
struct IMUState
{
  std::array<double, 3> linear_acceleration = {0.0, 0.0, 0.0};  // 线性加速度 (m/s²)
  std::array<double, 3> angular_velocity = {0.0, 0.0, 0.0};     // 角速度 (rad/s)
  std::array<double, 4> orientation = {0.0, 0.0, 0.0, 1.0};     // 四元数 (x, y, z, w)
};

// 全身广义状态
struct GeneralizedState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<double> ang_vel_b;
  Vec3<double> lin_acc_b;
  Quat<double> orientation_b;
  Vec3<double> rpy;
  RotMat<double> rotation_b2w;// 机身系转到世界系的旋转矩阵
  RotMat<double> rotation_w2b;// 世界系转到机身系的旋转矩阵
};

// 指令（包含速度指令、高度指令以及跳跃相关指令）
struct Command
{
  std::array<double, 3> cmd_vel = {0.0, 0.0, 0.0};  // 速度指令
  double cmd_height = 0.0;                             // 高度指令
  // 跳跃阶段 one-hot（例如：0-NORMAL，其余为不同跳跃阶段）
  // 这里固定为 5 维，对应训练时的 phase_onehot 维度
  std::array<double, 5> jump_phase_onehot = {1.0, 0.0, 0.0, 0.0, 0.0};
  // 跳跃高度（作为策略观察的一部分）
  double jump_height = 0.0;
};

// 机器人状态数据结构（暴露给状态机）
struct RobotState
{
  // 关节状态
  std::vector<JointState> joints;

  // 传感器状态
  IMUState imu;

  // 广义状态
  GeneralizedState body_state;

  // 运动指令
  Command command;

  // 时间信息
  double timestamp = 0.0;        // 时间戳 (s)
  double period = 0.0;            // 控制周期 (s)

  // 计算函数：计算各种状态表示
  void run();

  // 辅助函数：根据关节名称查找关节状态
  JointState* findJoint(const std::string& joint_name);
  const JointState* findJoint(const std::string& joint_name) const;

  // 辅助函数：获取所有关节的输出力矩
  std::vector<double> getOutputTorques() const;

  // 辅助函数：获取所有关节的位置
  std::vector<double> getPositions() const;

  // 辅助函数：获取所有关节的速度
  std::vector<double> getVelocities() const;

  // 辅助函数：获取所有关节的力矩
  std::vector<double> getEfforts() const;
};

}  // namespace robot_locomotion

#endif  // TEMPLATE_ROS2_CONTROLLER__ROBOT_STATE_HPP_

