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

#include "fsm/states/state_rl.hpp"
#include "fsm/state_machine.hpp"
#include "rcl/time.h"
#include "rclcpp/create_timer.hpp"
#include "robot_state/robot_state.hpp"
#include "tensorrt_cuda/tensorrt_inference.hpp"
#include "utils/timeMarker.h"
#include <algorithm>

namespace robot_locomotion {

StateRL::StateRL(StateMachine *state_machine, rclcpp::Logger logger)
    : StateBase(state_machine, logger) {
  // 初始化参数默认值（如果未从YAML配置，使用这些值）
  // 默认值：前4个关节使用不同的stiffness/damping，后2个轮子为0
  float default_stiffness[8] = {0.0f};
  float default_damping[8] = {0.0f};
  float default_action_scale[8] = {1.0f};
  float default_output_max[8] = {0.0f};
  float default_output_min[8] = {0.0f};
  float default_bias[8] = {0.0f};

  for (int i = 0; i < 8; i++) {
    this->params_.joint_stiffness[i] = default_stiffness[i];
    this->params_.joint_damping[i] = default_damping[i];
    this->params_.joint_action_scale[i] = default_action_scale[i];
    this->params_.joint_output_max[i] = default_output_max[i];
    this->params_.joint_output_min[i] = default_output_min[i];
    this->params_.joint_bias[i] = default_bias[i];
  }

  this->params_.ang_vel_scale = 1.0f;
  this->params_.dof_pos_scale = 1.0f;
  this->params_.dof_vel_scale = 1.0f;
  this->params_.cmd_scale = 1.0f;
  this->params_.height_scale = 1.0f;
  
  // 使用 Lambda 捕获 this 指针
  this->actuate[DofCtrlType::P] = [this]() { this->_P_actuate(); };
  this->actuate[DofCtrlType::V] = [this]() { this->_V_actuate(); };
  this->actuate[DofCtrlType::T] = [this]() { this->_T_actuate(); };
  // 配置关节执行模式
  this->dof_mode = DofCtrlType::P;

  // 初始化PID控制器（使用数组参数）
  for (int i = 0; i < 8; i++) {
      this->pid_controllers[i].getMicroTick_regist(getSystemTime);
      this->pid_controllers[i].PID_Init(Common, 0);

    // 使用配置的stiffness、damping和输出限制
    this->pid_controllers[i].Params_Config(
        PID_Mode::IS_PD, params_.joint_stiffness[i], params_.joint_damping[i],
        1e6, params_.joint_output_max[i], params_.joint_output_min[i]);
      this->pid_controllers[i].d_of_current = true;
  }
}

void StateRL::enter(const RobotState &robot_state, const rclcpp::Time &time) {
  (void)robot_state;
  (void)time;
  RCLCPP_INFO(logger_, "Entering RL state");
  // 启动 RL 推理器
  if (state_machine_ && state_machine_->isRLInferenceInitialized()) {
    state_machine_->startRLInference();
  }

  this->stop_update_ = false;
  this->threadRunning = true;

  // 根据模式选择调度方式
  if (mode_ == 2) {
    RCLCPP_INFO(logger_, "Lowlevel inline mode (no thread/timer)");
    return;
  }

  if (mode_ == 1) {
    // 确保使用 lowlevel_period_us_ 设置 period_microseconds_
    if (period_microseconds_ == 0 && lowlevel_period_us_ > 0) {
      period_microseconds_ = lowlevel_period_us_;
    }
    if (node_ && period_microseconds_ > 0) {
      auto clock = std::make_shared<rclcpp::Clock>(timer_clock_type_);
      ros_timer_ = rclcpp::create_timer(
          node_->get_node_base_interface(), node_->get_node_timers_interface(),
          clock,
          rclcpp::Duration::from_nanoseconds(period_microseconds_ * 1000),
          std::bind(&StateRL::onLowlevelTimer, this), nullptr);
      RCLCPP_INFO(logger_,
                  "Lowlevel control started with ROS2 timer (mode=1, clock=%d, "
                  "period=%lld us, freq=%.1f Hz)",
                  static_cast<int>(timer_clock_type_), period_microseconds_,
                  1e6 / static_cast<double>(period_microseconds_));
      return;
    } else {
      RCLCPP_WARN(
          logger_,
          "Lowlevel mode=1: period not set yet (period_us=%lld, "
          "lowlevel_period_us=%lld), will create timer after first run()",
          period_microseconds_, lowlevel_period_us_);
      // 保持 mode_=1，等待 run() 中设置 period 后再创建定时器
    }
  }

  // 默认：线程 + 系统时间
  if (this->thread_first_) {
    this->actuate_thread_ = std::thread(&StateRL::lowlevelThreadLoop, this);
    this->thread_first_ = false;
  }
  RCLCPP_INFO(logger_,
              "Lowlevel control started with system time thread (mode=0)");
}

void StateRL::run(RobotState &robot_state, const rclcpp::Time &time,
                  const rclcpp::Duration &period) {
  // 若模式为定时器且尚未创建定时器，则在首次 run 时创建
  if (mode_ == 1 && node_) {
    if (period_microseconds_ == 0) {
      period_microseconds_ = (lowlevel_period_us_ > 0)
                                 ? lowlevel_period_us_
                                 : (period.nanoseconds() / 1000);
    }
    if (period_microseconds_ > 0 && !ros_timer_) {
      auto clock = std::make_shared<rclcpp::Clock>(timer_clock_type_);
      ros_timer_ = rclcpp::create_timer(
          node_->get_node_base_interface(), node_->get_node_timers_interface(),
          clock,
          rclcpp::Duration::from_nanoseconds(period_microseconds_ * 1000),
          std::bind(&StateRL::onLowlevelTimer, this), nullptr);
      RCLCPP_INFO(logger_,
                  "Lowlevel timer (mode=1) created in run(), period: %lld us "
                  "(freq=%.1f Hz)",
                  period_microseconds_,
                  1e6 / static_cast<double>(period_microseconds_));
    }
  }
  
  // 更新线程安全的 robot_state 副本（供低层控制线程使用）
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state_copy_ = robot_state;
  }
  
  // 如果推理器未初始化，使用默认值
  if (!state_machine_ || !state_machine_->isRLInferenceInitialized()) {
    // 默认行为：所有力矩设为0
    for (auto &joint : robot_state.joints) {
      joint.output_torque = 0.0;
    }
    return;
  }

  // 准备输入数据：将机器人状态转换为模型输入
  std::vector<float> model_input;

  // 计算重力
  Vec3<double> projected_gravity =
      robot_state.body_state.rotation_w2b * Vec3<double>(0.0, 0.0, -1.);
  
  // angVel
  for (const auto &angVel : robot_state.body_state.ang_vel_b) {
    model_input.push_back(static_cast<float>(angVel));
  }

  // pg
  for (const auto &pg : projected_gravity) {
    model_input.push_back(static_cast<float>(pg));
  }

  // 添加关节位置
  for (const auto &joint : robot_state.joints) {
    if (joint.name.find("spring") == std::string::npos) {
      // if (joint.name.find("wheel") == std::string::npos){
      model_input.push_back(static_cast<float>(joint.position));
      // }
      // else{
      //   model_input.push_back(0.f);
      // }
    }
  }
  
  // 添加关节速度
  for (const auto &joint : robot_state.joints) {
    if (joint.name.find("spring") == std::string::npos) {
      model_input.push_back(static_cast<float>(joint.velocity));
    }
  }

  // 添加指令（此处不基于时间调整）
  for (const auto &cmd_vel : robot_state.command.cmd_vel) {
    model_input.push_back(static_cast<float>(cmd_vel));
  }
  // 使用从话题订阅获取的高度指令（不再硬编码）
  model_input.push_back(static_cast<float>(robot_state.command.cmd_height));
  
  // 添加上一次的模型输出
  for (const auto &la : this->last_actions_) {
    model_input.push_back(la);
  }

  // 设置输入数据（触发推理）
  state_machine_->setRLInput(model_input);

  // 获取推理输出
  std::vector<float> model_output;
  if (state_machine_->getRLOutput(model_output)) {
    // 将模型输出转换为关节力矩
    size_t output_size =
        std::min(model_output.size(), robot_state.joints.size());
    
    for (size_t i = 0; i < output_size; ++i) {
      this->desired_pos[i] = this->params_.joint_action_scale[i] *
                                 static_cast<double>(model_output[i]) +
                             this->params_.default_dof_pos[i];
      this->last_actions_[i] = model_output[i];
    }
  }

  if (mode_ == 2) {
    this->simplelowlevelCallbask(robot_state, time);
  }

  // 输出力矩（从线程安全的 torque 数组中读取，并添加偏置）
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    for (int i = 0; i < 8; i++) {
      robot_state.joints[i].output_torque = this->torque[i] + this->params_.joint_bias[i];
      }
  }
  // long long _start_time = getSystemTime();
  // // 计算并打印两次执行之间的时间间隔
  // if (this->last_execution_time_ > 0)
  // {
  //   long long time_diff = _start_time - this->last_execution_time_;
  //   RCLCPP_INFO(logger_, "state_rl execution interval: %lld us (%.3f ms)",
  //               time_diff, time_diff / 1000.0);
  // }
  // this->last_execution_time_ = _start_time;
}

void StateRL::exit(const RobotState &robot_state, const rclcpp::Time &time) {
  (void)robot_state;
  (void)time;
  RCLCPP_INFO(logger_, "Exiting RL state");
  // 停止 RL 推理器
  if (state_machine_) {
    state_machine_->stopRLInference();
  }
  this->stop_update_ = true;

  // 停止 ROS2 定时器
  if (ros_timer_) {
    ros_timer_->cancel();
    ros_timer_.reset();
  }

  // 停止独立线程
  this->threadRunning = false;
}

void StateRL::setMode(int mode, rcl_clock_type_t clock_type, double freq_hz,
                      rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
  mode_ = mode;
  timer_clock_type_ = clock_type;
  node_ = node;
  if (freq_hz > 0) {
    lowlevel_period_us_ = static_cast<long long>(1e6 / freq_hz);
    period_microseconds_ = lowlevel_period_us_;
  }
}

void StateRL::setJointParams(const std::vector<double> &stiffness,
                              const std::vector<double> &damping,
                              const std::vector<double> &action_scale,
                              const std::vector<double> &output_max,
                              const std::vector<double> &output_min,
                              const std::vector<double> &bias,
                              const std::vector<double> &default_dof_pos) {
  // 更新参数数组（最多8个关节）
  size_t max_joints = std::min(static_cast<size_t>(8), stiffness.size());
  max_joints = std::min(max_joints, damping.size());
  max_joints = std::min(max_joints, action_scale.size());
  max_joints = std::min(max_joints, output_max.size());
  max_joints = std::min(max_joints, output_min.size());
  max_joints = std::min(max_joints, bias.size());
  max_joints = std::min(max_joints, default_dof_pos.size());

  for (size_t i = 0; i < max_joints; i++) {
    params_.joint_stiffness[i] = static_cast<float>(stiffness[i]);
    params_.joint_damping[i] = static_cast<float>(damping[i]);
    params_.joint_action_scale[i] = static_cast<float>(action_scale[i]);
    params_.joint_output_max[i] = static_cast<float>(output_max[i]);
    params_.joint_output_min[i] = static_cast<float>(output_min[i]);
    params_.joint_bias[i] = static_cast<float>(bias[i]);
    params_.default_dof_pos[i] = static_cast<float>(default_dof_pos[i]);
  }

  // 更新PID控制器参数（前6个关节）
  for (int i = 0; i < 6 && i < static_cast<int>(max_joints); i++) {
    pid_controllers[i].Params_Config(
        PID_Mode::IS_PD, params_.joint_stiffness[i], params_.joint_damping[i],
        1e6, params_.joint_output_max[i], params_.joint_output_min[i]);
  }

  RCLCPP_INFO(logger_, "Joint parameters updated for %zu joints", max_joints);
}

void StateRL::simplelowlevelCallbask(RobotState &robot_state,
                                     const rclcpp::Time &time) {
  static float last_enable_time = 0;
  double period_s =
      (lowlevel_period_us_ > 0) ? (lowlevel_period_us_ / 1e6) : 0.002;
  if (time.seconds() - last_enable_time > period_s) {
    for (int i = 0; i < 6; i++) {
      if (i < static_cast<int>(robot_state.joints.size())) {
        if (i < 4) {
          this->torque[i] =
              this->params_.joint_stiffness[i] *
                  (this->desired_pos[i] - robot_state.joints[i].position) +
              this->params_.joint_damping[i] * robot_state.joints[i].velocity;
        }
      } else {
        // 轮子关节：使用配置的damping
        this->torque[i] =
            this->desired_pos[i] +
            this->params_.joint_damping[i] * robot_state.joints[i].velocity;
      }
      // 使用配置的输出限制（偏置在最终输出时添加）
      if (this->torque[i] > this->params_.joint_output_max[i]) {
        this->torque[i] = this->params_.joint_output_max[i];
      }
      if (this->torque[i] < this->params_.joint_output_min[i]) {
        this->torque[i] = this->params_.joint_output_min[i];
      }
    }
  }
  last_enable_time = time.seconds();
}

void StateRL::doLowlevelStep() {
  // 公共的低层控制执行逻辑
  long long _start_time = getSystemTime();

  if (!this->stop_update_) {
    this->actuate[this->dof_mode]();
  }

  // 计算并打印两次执行之间的时间间隔
  // if (this->last_execution_time_ > 0)
  // {
  //   long long time_diff = _start_time - this->last_execution_time_;
  //   RCLCPP_INFO(logger_, "Lowlevel execution interval: %lld us (%.3f ms)",
  //               time_diff, time_diff / 1000.0);
  // }
  this->last_execution_time_ = _start_time;
}

void StateRL::onLowlevelTimer() {
  // ROS2 定时器回调函数（模式1：定时器调度）
  doLowlevelStep();
}

void StateRL::lowlevelThreadLoop() {
  // 线程循环函数（模式0：独立线程调度，使用系统时间进行频率控制）
  while (this->threadRunning) {
    long long _start_time = getSystemTime();

    // 执行低层控制逻辑
    doLowlevelStep();

    // 使用系统时间进行频率控制（依据配置频率，默认 500Hz）
    long long wait_us = (lowlevel_period_us_ > 0)
                            ? lowlevel_period_us_
                            : (long long)(0.002 * 1000000);
    absoluteWait(_start_time, wait_us);
  }
  this->threadRunning = false;
}

void StateRL::_P_actuate() {
  // 从线程安全的副本中获取 robot_state
  RobotState robot_state;
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state = robot_state_copy_;
  }
  
  // 计算 PID 输出
  double local_torque[6] = {0};
  for (int i = 0; i < 6; i++) {
    if (i < static_cast<int>(robot_state.joints.size())) {
      this->pid_controllers[i].target = this->desired_pos[i];
      this->pid_controllers[i].current = robot_state.joints[i].position;
      this->pid_controllers[i].Adjust(0, robot_state.joints[i].velocity);
      local_torque[i] = this->pid_controllers[i].out;
      if (i > 3) {
        local_torque[i] += this->desired_pos[i];
      }
      // 应用输出限制（偏置在最终输出时添加）
      if (local_torque[i] > this->pid_controllers[i].Output_Max) {
        local_torque[i] = this->pid_controllers[i].Output_Max;
      }
      if (local_torque[i] < this->pid_controllers[i].Output_Min) {
        local_torque[i] = this->pid_controllers[i].Output_Min;
      }
    }
  }
  
  // 线程安全地更新 torque 数组
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    for (int i = 0; i < 6; i++) {
      this->torque[i] = local_torque[i];
    }
  }
}

void StateRL::_V_actuate() {
  // 从线程安全的副本中获取 robot_state
  RobotState robot_state;
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state = robot_state_copy_;
  }
  
  // 计算速度控制输出
  double local_torque[6] = {0};
  for (int i = 0; i < 6; i++) {
    if (i < static_cast<int>(robot_state.joints.size())) {
      // TODO: 实现速度控制逻辑
      // 示例：this->pid_controllers[i].target = desired_velocity;
      //      this->pid_controllers[i].current = robot_state.joints[i].velocity;
      //      this->pid_controllers[i].Adjust(0, 0);
      //      local_torque[i] = this->pid_controllers[i].out;
    }
  }
  
  // 线程安全地更新 torque 数组
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    for (int i = 0; i < 6; i++) {
      this->torque[i] = local_torque[i];
    }
  }
}

void StateRL::_T_actuate() {
  // 从线程安全的副本中获取 robot_state
  RobotState robot_state;
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state = robot_state_copy_;
  }
  
  // 计算力矩控制输出
  double local_torque[6] = {0};
  for (int i = 0; i < 6; i++) {
    if (i < static_cast<int>(robot_state.joints.size())) {
      // TODO: 实现力矩控制逻辑
      // 示例：直接使用期望力矩
      //      local_torque[i] = this->desired_torque[i];
      // 或者根据当前状态进行力矩补偿
      //      local_torque[i] = this->desired_torque[i] + compensation;
    }
  }
  
  // 线程安全地更新 torque 数组
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    for (int i = 0; i < 6; i++) {
      this->torque[i] = local_torque[i];
    }
  }
}

} // namespace robot_locomotion
