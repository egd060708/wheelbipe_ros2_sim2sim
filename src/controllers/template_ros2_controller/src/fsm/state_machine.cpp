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

namespace robot_locomotion
{

StateMachine::StateMachine(rclcpp::Logger logger)
  : current_state_(ControllerState::INIT)
  , target_state_(ControllerState::IDLE)
  , logger_(logger)
  , num_joints_(0)
{
  state_entry_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void StateMachine::update(RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period)
{
  (void)period;
  
  // 更新关节数量（仅在第一次）
  if (num_joints_ == 0 && !robot_state.joints.empty()) {
    num_joints_ = robot_state.joints.size();
  }

  // 处理状态转换
  handleStateTransition(robot_state, time);

  // 根据当前状态处理
  switch (current_state_) {
    case ControllerState::INIT:
      processInitState(robot_state, time);
      break;
    case ControllerState::IDLE:
      processIdleState(robot_state, time);
      break;
    case ControllerState::RL:
      processRLState(robot_state, time);
      break;
    case ControllerState::STANDING:
      processStandingState(robot_state, time);
      break;
    case ControllerState::WALKING:
      processWalkingState(robot_state, time);
      break;
    case ControllerState::RUNNING:
      processRunningState(robot_state, time);
      break;
    case ControllerState::ERROR:
      processErrorState(robot_state, time);
      break;
    case ControllerState::EMERGENCY_STOP:
      processEmergencyStopState(robot_state, time);
      break;
  }
}

void StateMachine::handleStateTransition(const RobotState& robot_state, const rclcpp::Time& time)
{
  (void)robot_state;
  
  ControllerState next_state = current_state_;

  switch (current_state_) {
    case ControllerState::INIT:
      // 初始化完成后进入空闲状态
      if ((time - state_entry_time_).seconds() > 0.1) {
        next_state = ControllerState::IDLE;
      }
      break;

    case ControllerState::IDLE:
      // 空闲状态后自动进入强化学习状态
      if ((time - state_entry_time_).seconds() > 1.0) {
        next_state = ControllerState::RL;
      }
      // 如果设置了目标状态，也可以转换到目标状态
      else if (target_state_ != ControllerState::IDLE && target_state_ != ControllerState::INIT) {
        next_state = target_state_;
      }
      break;

    case ControllerState::RL:
      // RL 状态：如果设置了目标状态（且不是 IDLE），可以转换
      // 避免因为 target_state_ 是 IDLE 而循环切换
      if (target_state_ != ControllerState::RL && 
          target_state_ != ControllerState::INIT) {
        next_state = target_state_;
      }
      break;

    case ControllerState::STANDING:
    case ControllerState::WALKING:
    case ControllerState::RUNNING:
      // 如果目标状态改变，可以转换
      if (target_state_ != current_state_ && target_state_ != ControllerState::INIT) {
        next_state = target_state_;
      }
      // 错误检测：如果检测到异常，进入错误状态
      // 这里可以添加错误检测逻辑
      break;

    case ControllerState::ERROR:
      // 错误状态需要手动重置
      break;

    case ControllerState::EMERGENCY_STOP:
      // 紧急停止状态需要手动重置
      break;
  }

  // 执行状态转换
  if (next_state != current_state_) {
    const char* next_state_name = [next_state]() -> const char* {
      switch (next_state) {
        case ControllerState::INIT: return "INIT";
        case ControllerState::IDLE: return "IDLE";
        case ControllerState::RL: return "RL";
        case ControllerState::STANDING: return "STANDING";
        case ControllerState::WALKING: return "WALKING";
        case ControllerState::RUNNING: return "RUNNING";
        case ControllerState::ERROR: return "ERROR";
        case ControllerState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
      }
    }();
    RCLCPP_INFO(logger_, "State transition: %s -> %s",
      getStateName().c_str(), next_state_name);
    
    // 离开 RL 状态时停止推理器
    if (current_state_ == ControllerState::RL) {
      stopRLInference();
    }
    
    // 进入 RL 状态时启动推理器并更新目标状态
    if (next_state == ControllerState::RL) {
      if (isRLInferenceInitialized()) {
        startRLInference();
      }
      // 更新目标状态为 RL，避免循环切换
      target_state_ = ControllerState::RL;
    }
    
    current_state_ = next_state;
    state_entry_time_ = time;
  }
}

const std::vector<double> StateMachine::getOutputTorques(const RobotState& robot_state) const
{
  std::vector<double> torques;
  torques.reserve(robot_state.joints.size());
  for (const auto& joint : robot_state.joints) {
    torques.push_back(joint.output_torque);
  }
  return torques;
}

std::string StateMachine::getStateName() const
{
  switch (current_state_) {
    case ControllerState::INIT: return "INIT";
    case ControllerState::IDLE: return "IDLE";
    case ControllerState::RL: return "RL";
    case ControllerState::STANDING: return "STANDING";
    case ControllerState::WALKING: return "WALKING";
    case ControllerState::RUNNING: return "RUNNING";
    case ControllerState::ERROR: return "ERROR";
    case ControllerState::EMERGENCY_STOP: return "EMERGENCY_STOP";
    default: return "UNKNOWN";
  }
}

void StateMachine::setTargetState(ControllerState target_state)
{
  if (target_state != ControllerState::INIT && target_state != ControllerState::ERROR) {
    target_state_ = target_state;
  }
}

void StateMachine::reset()
{
  stopRLInference();
  current_state_ = ControllerState::INIT;
  target_state_ = ControllerState::IDLE;
  state_entry_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

bool StateMachine::initializeRLInference(const std::string& engine_model_path, int inference_frequency_hz)
{
  if (rl_inference_) {
    RCLCPP_WARN(logger_, "RL inference already initialized");
    return true;
  }

  rl_inference_ = std::make_unique<TensorRTInference>(logger_);
  if (!rl_inference_->initialize(engine_model_path, inference_frequency_hz)) {
    RCLCPP_ERROR(logger_, "Failed to initialize RL inference");
    rl_inference_.reset();
    return false;
  }

  RCLCPP_INFO(logger_, "RL inference initialized (not started yet)");
  return true;
}

void StateMachine::startRLInference()
{
  if (!rl_inference_) {
    RCLCPP_WARN(logger_, "RL inference not initialized, cannot start");
    return;
  }

  if (rl_inference_->isRunning()) {
    RCLCPP_WARN(logger_, "RL inference already running");
    return;
  }

  rl_inference_->start();
  RCLCPP_INFO(logger_, "RL inference started");
}

void StateMachine::stopRLInference()
{
  if (rl_inference_ && rl_inference_->isRunning()) {
    rl_inference_->stop();
    RCLCPP_INFO(logger_, "RL inference stopped (but not destroyed)");
  }
}

bool StateMachine::isRLInferenceInitialized() const
{
  return rl_inference_ != nullptr && rl_inference_->isInitialized();
}

}  // namespace robot_locomotion

