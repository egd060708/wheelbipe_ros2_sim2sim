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
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rcl/time.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace robot_locomotion
{

// 前向声明
struct RobotState;
class TensorRTInference;
class StateBase;

// 状态机状态枚举
enum class ControllerState
{
  INIT = 0,           // 初始化状态
  IDLE,               // 空闲状态
  RL,                 // 强化学习状态
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
  StateMachine(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node = nullptr);
  ~StateMachine();

  // 状态机更新函数，根据当前状态和机器人状态进行状态转换和输出计算
  void update(RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period);

  // 获取当前状态
  ControllerState getCurrentState() const { return current_state_; }

  // 获取状态名称
  std::string getStateName() const;
  std::string getStateName(ControllerState state) const;

  // 设置目标状态（用于外部控制）
  void setTargetState(ControllerState target_state);

  // 获取输出力矩（从 RobotState 获取）
  const std::vector<double> getOutputTorques(const RobotState& robot_state) const;

  // 重置状态机
  void reset();

  // 初始化 RL 推理器（只初始化，不启动）
  bool initializeRLInference(const std::string& engine_model_path, int inference_frequency_hz);

  // 启动 RL 推理器（在进入 RL 状态时调用）
  void startRLInference();

  // 停止 RL 推理器（在离开 RL 状态时调用）
  void stopRLInference();

  // 检查 RL 推理器是否已初始化
  bool isRLInferenceInitialized() const;

  // 检查 RL 推理器是否正在运行
  bool isRLInferenceRunning() const;

  // 设置 RL 推理器输入
  void setRLInput(const std::vector<float>& input_data);

  // 获取 RL 推理器输出
  bool getRLOutput(std::vector<float>& output_data);

  // 获取目标状态
  ControllerState getTargetState() const { return target_state_; }

  // 设置调度模式与定时器时钟
  void setTimingConfig(int inference_mode, int lowlevel_mode,
                       const std::string& inference_clock_type,
                       const std::string& lowlevel_clock_type);
  void setLowlevelFrequency(double freq_hz);

  // TensorRT 推理器（用于 RL 状态）
  std::unique_ptr<TensorRTInference> rl_inference_;

protected:
  // 状态转换逻辑
  ControllerState handleStateTransition(const RobotState& robot_state, const rclcpp::Time& time);

  // 切换状态
  void changeState(ControllerState new_state, const RobotState& robot_state, const rclcpp::Time& time);

  // 初始化所有状态对象
  void initializeStates();

  ControllerState current_state_;
  ControllerState target_state_;
  rclcpp::Logger logger_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;  // 用于创建定时器
  rclcpp::Time state_entry_time_;
  size_t num_joints_;
  bool first_update_;

  // 状态对象管理
  std::map<ControllerState, std::unique_ptr<StateBase>> states_;
  StateBase* current_state_obj_;

  // // TensorRT 推理器（用于 RL 状态）
  // std::unique_ptr<TensorRTInference> rl_inference_;
  
  // 调度模式与时钟
  int inference_mode_ = 0;
  int lowlevel_mode_ = 0;
  rcl_clock_type_t inference_clock_type_ = RCL_ROS_TIME;
  rcl_clock_type_t lowlevel_clock_type_ = RCL_ROS_TIME;
  double lowlevel_frequency_hz_ = 500.0;  // 默认 500Hz
};

}  // namespace robot_locomotion

#endif  // TEMPLATE_ROS2_CONTROLLER__STATE_MACHINE_HPP_

