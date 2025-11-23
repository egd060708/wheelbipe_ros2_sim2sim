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
#include "tensorrt_cuda/tensorrt_inference.hpp"
#include "robot_state/robot_state.hpp"
#include <algorithm>

namespace robot_locomotion
{

void StateMachine::processRLState(RobotState& robot_state, const rclcpp::Time& time)
{
  (void)time;
  
  // 如果推理器未初始化，使用默认值
  if (!rl_inference_ || !rl_inference_->isInitialized() || !rl_inference_->isRunning()) {
    // 默认行为：所有力矩设为0
    for (auto& joint : robot_state.joints) {
      joint.output_torque = 0.0;
    }
    return;
  }

  // 准备输入数据：将机器人状态转换为模型输入
  // 这里需要根据实际模型输入格式调整
  std::vector<float> model_input;
  
  // 添加关节位置
  for (const auto& joint : robot_state.joints) {
    model_input.push_back(static_cast<float>(joint.position));
  }
  
  // 添加关节速度
  for (const auto& joint : robot_state.joints) {
    model_input.push_back(static_cast<float>(joint.velocity));
  }
  
  // 添加关节力矩
  for (const auto& joint : robot_state.joints) {
    model_input.push_back(static_cast<float>(joint.effort));
  }
  
  // 添加 IMU 数据
  for (size_t i = 0; i < 3; ++i) {
    model_input.push_back(static_cast<float>(robot_state.imu.linear_acceleration[i]));
  }
  for (size_t i = 0; i < 3; ++i) {
    model_input.push_back(static_cast<float>(robot_state.imu.angular_velocity[i]));
  }
  for (size_t i = 0; i < 4; ++i) {
    model_input.push_back(static_cast<float>(robot_state.imu.orientation[i]));
  }

  // 设置输入数据（触发推理）
  rl_inference_->setInput(model_input);

  // 获取推理输出
  std::vector<float> model_output;
  if (rl_inference_->getOutput(model_output)) {
    // 将模型输出转换为关节力矩
    // 假设模型输出直接对应关节力矩
    size_t output_size = std::min(model_output.size(), robot_state.joints.size());
    for (size_t i = 0; i < output_size; ++i) {
      robot_state.joints[i].output_torque = static_cast<double>(model_output[i]);
    }
    
    // 如果输出数量不匹配，剩余关节设为0
    for (size_t i = output_size; i < robot_state.joints.size(); ++i) {
      robot_state.joints[i].output_torque = 0.0;
    }
  } else {
    // 如果获取输出失败，使用默认值
    for (auto& joint : robot_state.joints) {
      joint.output_torque = 0.0;
    }
  }
}

}  // namespace robot_locomotion

