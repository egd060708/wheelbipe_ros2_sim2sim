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

#include "fsm/state_machine.hpp"
#include "fsm/state_base.hpp"
#include "fsm/states/state_init.hpp"
#include "fsm/states/state_idle.hpp"
#include "fsm/states/state_rl.hpp"
#include "tensorrt_cuda/tensorrt_inference.hpp"
#include "robot_state/robot_state.hpp"
#include "rclcpp/clock.hpp"
#include "rcl/time.h"

namespace robot_locomotion
{

StateMachine::StateMachine(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : current_state_(ControllerState::INIT)
  , target_state_(ControllerState::IDLE)
  , logger_(logger)
  , node_(node)
  , num_joints_(0)
  , first_update_(true)
  , current_state_obj_(nullptr)
{
  state_entry_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  initializeStates();
  // 设置初始状态对象（不在这里调用 enter，在第一次 update 时调用）
  if (states_.find(ControllerState::INIT) != states_.end()) {
    current_state_obj_ = states_[ControllerState::INIT].get();
  }
}

StateMachine::~StateMachine()
{
  // 调用当前状态的 exit（如果存在）
  if (current_state_obj_) {
    RobotState empty_state;
    current_state_obj_->exit(empty_state, rclcpp::Time(0, 0, RCL_ROS_TIME));
  }
  // 停止 RL 推理器
  stopRLInference();
  // 状态对象会自动清理（通过 unique_ptr）
}

void StateMachine::update(RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // 更新关节数量（仅在第一次）
  if (num_joints_ == 0 && !robot_state.joints.empty()) {
    num_joints_ = robot_state.joints.size();
  }

  // 如果是第一次调用且当前状态对象存在，调用 enter
  if (first_update_ && current_state_obj_) {
    current_state_obj_->enter(robot_state, time);
    first_update_ = false;
  }

  // 处理状态转换
  ControllerState next_state = handleStateTransition(robot_state, time);
  
  // 如果状态发生变化，执行状态切换
  if (next_state != current_state_) {
    changeState(next_state, robot_state, time);
  }
  
  // 注意：如果使用 ROS2 定时器，不需要在这里更新，定时器会自动调度
  
  // 运行当前状态
  if (current_state_obj_) {
    current_state_obj_->run(robot_state, time, period);
  }
}

ControllerState StateMachine::handleStateTransition(const RobotState& robot_state, const rclcpp::Time& time)
{
  (void)robot_state;
  
  ControllerState next_state = current_state_;

  switch (current_state_) {
    case ControllerState::INIT:
      // 初始化完成后进入空闲状态
      if ((time - state_entry_time_).seconds() > 0.01) {
        next_state = ControllerState::IDLE;
      }
      break;

    case ControllerState::IDLE:
      // 空闲状态后自动进入强化学习状态
      if ((time - state_entry_time_).seconds() > 1.) {
        this->setTargetState(ControllerState::RL);
      }
      // 如果设置了目标状态，也可以转换到目标状态
      if (target_state_ != ControllerState::IDLE && target_state_ != ControllerState::INIT) {
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
  }

  return next_state;
}

void StateMachine::changeState(ControllerState new_state, const RobotState& robot_state, const rclcpp::Time& time)
{
  if (new_state == current_state_) {
    return;
  }

  // 调用旧状态的 exit
  if (current_state_obj_) {
    current_state_obj_->exit(robot_state, time);
  }

  // 更新状态
  ControllerState old_state = current_state_;
  current_state_ = new_state;
  state_entry_time_ = time;

  // 更新状态对象指针
  if (states_.find(new_state) != states_.end()) {
    current_state_obj_ = states_[new_state].get();
  } else {
    current_state_obj_ = nullptr;
    RCLCPP_ERROR(logger_, "State object not found for state: %d", static_cast<int>(new_state));
  }

  // 记录状态转换
  RCLCPP_INFO(logger_, "State transition: %s -> %s",
    getStateName(old_state).c_str(), getStateName(new_state).c_str());

  // 调用新状态的 enter
  if (current_state_obj_) {
    current_state_obj_->enter(robot_state, time);
  }
}

void StateMachine::initializeStates()
{
  // 初始化所有状态对象
  states_[ControllerState::INIT] = std::make_unique<StateInit>(this, logger_);
  states_[ControllerState::IDLE] = std::make_unique<StateIdle>(this, logger_);
  states_[ControllerState::RL] = std::make_unique<StateRL>(this, logger_);
  
  // 初始化后立即应用当前配置到 RL 状态
  if (states_.find(ControllerState::RL) != states_.end()) {
    StateRL* rl_state = dynamic_cast<StateRL*>(states_[ControllerState::RL].get());
    if (rl_state) {
      rl_state->setMode(lowlevel_mode_, lowlevel_clock_type_, lowlevel_frequency_hz_, node_);
    }
  }
}

std::string StateMachine::getStateName(ControllerState state) const
{
  switch (state) {
    case ControllerState::INIT: return "INIT";
    case ControllerState::IDLE: return "IDLE";
    case ControllerState::RL: return "RL";
    default: return "UNKNOWN";
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
  return getStateName(current_state_);
}

void StateMachine::setTargetState(ControllerState target_state)
{
  if (target_state != ControllerState::INIT) {
    target_state_ = target_state;
  }
}

void StateMachine::reset()
{
  stopRLInference();
  current_state_ = ControllerState::INIT;
  target_state_ = ControllerState::IDLE;
  state_entry_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  first_update_ = true;
  // 更新状态对象指针
  if (states_.find(ControllerState::INIT) != states_.end()) {
    current_state_obj_ = states_[ControllerState::INIT].get();
  }
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

  // 设置调度模式
  rl_inference_->setMode(inference_mode_, inference_clock_type_, node_);

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

bool StateMachine::isRLInferenceRunning() const
{
  return rl_inference_ != nullptr && rl_inference_->isRunning();
}

void StateMachine::setRLInput(const std::vector<float>& input_data)
{
  if (rl_inference_) {
    rl_inference_->setInput(input_data);
  }
}

bool StateMachine::getRLOutput(std::vector<float>& output_data)
{
  if (rl_inference_) {
    return rl_inference_->getOutput(output_data);
  }
  return false;
}

static rcl_clock_type_t parseClockType(const std::string& type_str)
{
  if (type_str == "ROS_TIME") {
    return RCL_ROS_TIME;
  } else if (type_str == "SYSTEM_TIME") {
    return RCL_SYSTEM_TIME;
  } else {  // 默认 STEADY
    return RCL_STEADY_TIME;
  }
}

void StateMachine::setTimingConfig(int inference_mode, int lowlevel_mode,
                                   const std::string& inference_clock_type,
                                   const std::string& lowlevel_clock_type)
{
  inference_mode_ = inference_mode;
  lowlevel_mode_ = lowlevel_mode;
  inference_clock_type_ = parseClockType(inference_clock_type);
  lowlevel_clock_type_ = parseClockType(lowlevel_clock_type);
  
  // 设置推理器的配置
  if (rl_inference_) {
    rl_inference_->setMode(inference_mode_, inference_clock_type_, node_);
  }
  
  // 设置 RL 状态的配置
  if (states_.find(ControllerState::RL) != states_.end()) {
    StateRL* rl_state = dynamic_cast<StateRL*>(states_[ControllerState::RL].get());
    if (rl_state) {
      rl_state->setMode(lowlevel_mode_, lowlevel_clock_type_, lowlevel_frequency_hz_, node_);
    }
  }
}

void StateMachine::setLowlevelFrequency(double freq_hz)
{
  lowlevel_frequency_hz_ = freq_hz;
  // 如果 StateRL 已创建，立即更新
  if (states_.find(ControllerState::RL) != states_.end()) {
    StateRL* rl_state = dynamic_cast<StateRL*>(states_[ControllerState::RL].get());
    if (rl_state) {
      rl_state->setMode(lowlevel_mode_, lowlevel_clock_type_, lowlevel_frequency_hz_, node_);
    }
  }
}

void StateMachine::setJointParams(const std::vector<double>& stiffness,
                                   const std::vector<double>& damping,
                                   const std::vector<double>& action_scale,
                                   const std::vector<double>& output_max,
                                   const std::vector<double>& output_min,
                                   const std::vector<double>& bias,
                                   const std::vector<double>& default_dof_pos)
{
  // 传递给 RL 状态
  if (states_.find(ControllerState::RL) != states_.end()) {
    StateRL* rl_state = dynamic_cast<StateRL*>(states_[ControllerState::RL].get());
    if (rl_state) {
      rl_state->setJointParams(stiffness, damping, action_scale, output_max, output_min, bias, default_dof_pos);
    }
  }
}

}  // namespace robot_locomotion

