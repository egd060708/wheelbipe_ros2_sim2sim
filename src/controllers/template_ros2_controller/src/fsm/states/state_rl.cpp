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

#include "fsm/states/state_rl.hpp"
#include "fsm/state_machine.hpp"
#include "tensorrt_cuda/tensorrt_inference.hpp"
#include "robot_state/robot_state.hpp"
#include <algorithm>
#include "utils/timeMarker.h"

namespace robot_locomotion
{

StateRL::StateRL(StateMachine* state_machine, rclcpp::Logger logger)
  : StateBase(state_machine, logger)
{
  // 初始化参数
  this->params_.joint_damping = -0.5f;
  this->params_.joint_stiffness = 30.f;
  this->params_.wheel_damping = -0.01f;
  this->params_.wheel_stiffness = 0.f;
  this->params_.joint_action_scale = 0.5f;
  this->params_.wheel_action_scale = 1.0f;
  this->params_.ang_vel_scale = 1.0f;
  this->params_.dof_pos_scale = 1.0f;
  this->params_.dof_vel_scale = 1.0f;
  this->params_.cmd_scale = 1.0f;
  this->params_.height_scale = 1.0f;
  this->params_.spring_force = 240.f;
  
  // 使用 Lambda 捕获 this 指针
  this->actuate[DofCtrlType::P] = [this]()
  { this->_P_actuate(); };
  this->actuate[DofCtrlType::V] = [this]()
  { this->_V_actuate(); };
  this->actuate[DofCtrlType::T] = [this]()
  { this->_T_actuate(); };
  // 配置关节执行模式
  this->dof_mode = DofCtrlType::P;

  for(int i=0;i<6;i++)
  {
    if(i<4){
      this->pid_controllers[i].getMicroTick_regist(getSystemTime);
      this->pid_controllers[i].PID_Init(Common, 0);
      this->pid_controllers[i].Params_Config(PID_Mode::IS_PD,
                                              params_.joint_stiffness,
                                              params_.joint_damping,
                                              1e6,39.9,-39.9);
      this->pid_controllers[i].d_of_current = true;
    }
    else{
      this->pid_controllers[i].getMicroTick_regist(getSystemTime);
      this->pid_controllers[i].PID_Init(Common, 0);
      this->pid_controllers[i].Params_Config(PID_Mode::IS_PD,
                                              params_.wheel_stiffness,
                                              params_.wheel_damping,
                                              1e6,4.99,-4.99);
      this->pid_controllers[i].d_of_current = true;
    }
  }
}

void StateRL::enter(const RobotState& robot_state, const rclcpp::Time& time)
{
  (void)robot_state;
  (void)time;
  RCLCPP_INFO(logger_, "Entering RL state");
  // 启动 RL 推理器
  if (state_machine_ && state_machine_->isRLInferenceInitialized()) {
    state_machine_->startRLInference();
  }
  
  this->stop_update_ = false;
  this->threadRunning = true;
  
  // 根据配置选择使用 ROS2 定时器或独立线程
  if (use_period_timing_ && node_) {
    // // 使用 ROS2 定时器（period 会在 run() 中设置）
    // // 如果 period 还未设置，等待 run() 调用
    // if (period_microseconds_ > 0) {
    //   ros_timer_ = node_->create_wall_timer(
    //     std::chrono::microseconds(period_microseconds_),
    //     std::bind(&StateRL::lowlevelCallback, this));
    //   RCLCPP_INFO(logger_, "Lowlevel control started with ROS2 timer (period: %lld us)", period_microseconds_);
    // } else {
    //   RCLCPP_WARN(logger_, "Lowlevel control: ROS2 timer will be created after period is set");
    // }
  } else {
    // 使用独立线程（系统时间）
    if (this->thread_first_)
    {
      this->actuate_thread_ = std::thread(&StateRL::_Run_Lowlevel, this);
      this->thread_first_ = false;
    }
    RCLCPP_INFO(logger_, "Lowlevel control started with system time thread");
  }
}

void StateRL::run(RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period)
{
  static float last_enable_time = 0.;
  // 更新 period（如果使用 period 进行频率计算）
  // if (use_period_timing_) {
  //   long long new_period_microseconds = period.nanoseconds() / 1000;
  //   if (new_period_microseconds != period_microseconds_) {
  //     period_microseconds_ = new_period_microseconds;
  //     // 如果定时器还未创建，现在创建它
  //     if (node_ && !ros_timer_ && period_microseconds_ > 0) {
  //       ros_timer_ = node_->create_wall_timer(
  //         std::chrono::microseconds(period_microseconds_),
  //         std::bind(&StateRL::lowlevelCallback, this));
  //       RCLCPP_INFO(logger_, "Lowlevel control ROS2 timer created (period: %lld us)", period_microseconds_);
  //     } else if (ros_timer_ && period_microseconds_ > 0) {
  //       // 如果定时器已存在但 period 改变了，需要重新创建
  //       ros_timer_->cancel();
  //       ros_timer_ = node_->create_wall_timer(
  //         std::chrono::microseconds(period_microseconds_),
  //         std::bind(&StateRL::lowlevelCallback, this));
  //       RCLCPP_INFO(logger_, "Lowlevel control ROS2 timer updated (period: %lld us)", period_microseconds_);
  //     }
  //   }
  // }
  
  // 更新线程安全的 robot_state 副本（供低层控制线程使用）
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state_copy_ = robot_state;
  }
  
  // 如果推理器未初始化，使用默认值
  if (!state_machine_ || !state_machine_->isRLInferenceInitialized()) {
    // 默认行为：所有力矩设为0
    for (auto& joint : robot_state.joints) {
      joint.output_torque = 0.0;
    }
    return;
  }

  // 准备输入数据：将机器人状态转换为模型输入
  std::vector<float> model_input;

  // 计算重力
  Vec3<double> projected_gravity = robot_state.body_state.rotation_b2w * Vec3<double>(0.0, 0.0, -1.);
  
  // angVel
  for(const auto& angVel: robot_state.body_state.ang_vel_b) {
    model_input.push_back(static_cast<float>(angVel));
  }

  // pg
  for(const auto& pg: projected_gravity) {
    model_input.push_back(static_cast<float>(pg));
  }

  // 添加关节位置
  for (const auto& joint : robot_state.joints) {
    if (joint.name.find("spring") == std::string::npos){
      model_input.push_back(static_cast<float>(joint.position));
    }
  }
  
  // 添加关节速度
  for (const auto& joint : robot_state.joints) {
    if (joint.name.find("spring") == std::string::npos){
      model_input.push_back(static_cast<float>(joint.velocity));
    }
  }

  // 添加指令
  robot_state.command.cmd_vel[0] = 2;
  for (const auto& cmd_vel : robot_state.command.cmd_vel) {
    model_input.push_back(static_cast<float>(cmd_vel));
  }
  robot_state.command.cmd_height = 0.2;
  model_input.push_back(static_cast<float>(robot_state.command.cmd_height));
  
  // 添加上一次的模型输出
  for (const auto& la: this->last_actions_) {
    model_input.push_back(la);
  }

  // 设置输入数据（触发推理）
  state_machine_->setRLInput(model_input);

  if (use_period_timing_)
  {
    if (time.seconds()-last_enable_time>0.02){
      state_machine_->rl_inference_->inferenceCallback();
      last_enable_time = time.seconds();
    }
  }

  // 获取推理输出
  std::vector<float> model_output;
  if (state_machine_->getRLOutput(model_output)) {
    // 将模型输出转换为关节力矩
    size_t output_size = std::min(model_output.size(), robot_state.joints.size());
    
    for (size_t i = 0; i < output_size; ++i) {
      if(i<4){
        // this->desired_pos[i] = this->params_.joint_action_scale*static_cast<double>(model_output[i])+this->params_.default_dof_pos[i];
      }
      else{
        this->desired_pos[i] = this->params_.wheel_action_scale*static_cast<double>(model_output[i]);
      }
      this->last_actions_[i] = model_output[i];
    }
  }

  if (use_period_timing_)
  {
    simplelowlevelCallbask(robot_state, time);
  }

  // 输出力矩（从线程安全的 torque 数组中读取）
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    for(int i=0;i<8;i++)
    {
      if(i<6){
        robot_state.joints[i].output_torque = this->torque[i];
      }
      else{
        robot_state.joints[i].output_torque = this->params_.spring_force;
      }
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

void StateRL::exit(const RobotState& robot_state, const rclcpp::Time& time)
{
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

void StateRL::setUsePeriodTiming(bool use_period, rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  use_period_timing_ = use_period;
  node_ = node;
}

void StateRL::simplelowlevelCallbask(RobotState& robot_state, const rclcpp::Time& time)
{
  static float last_enable_time = 0;
  if(time.seconds()-last_enable_time>0.005)
  {
    for (int i=0; i < 6; i++){
      if (i < static_cast<int>(robot_state.joints.size())) {
        if(i<4){
          if(this->desired_pos[i]>3.14)
          {
            this->desired_pos[i]=3.14;
          }
          else if(this->desired_pos[i]<-3.14)
          {
            this->desired_pos[i]=-3.14;
          }
          this->torque[i] = this->params_.joint_stiffness*(this->desired_pos[i]-robot_state.joints[i].position)
                            + this->params_.joint_damping*robot_state.joints[i].velocity;
        }
        else{
          this->torque[i] = this->desired_pos[i] + this->params_.wheel_damping*robot_state.joints[i].velocity;
        }
        if(i<4){
          if(this->torque[i]>39.9){
            this->torque[i] = 39.9;
          }
          if(this->torque[i]<-39.9){
            this->torque[i] = -39.9;
          }
        }
        else
        {
          if(this->torque[i]>4.99){
            this->torque[i] = 4.99;
          }
          if(this->torque[i]<-4.99){
            this->torque[i] = -4.99;
          }
        }
      }
    }
    last_enable_time = time.seconds();
  }
}

void StateRL::lowlevelCallback()
{
  // ROS2 定时器回调函数
  long long _start_time = getSystemTime();
  
  if (!this->stop_update_)
  {
    this->actuate[this->dof_mode]();
  }
  
  // 计算并打印两次执行之间的时间间隔
  // if (this->last_execution_time_ > 0)
  // {
  //   long long time_diff = _start_time - this->last_execution_time_;
  //   RCLCPP_INFO(logger_, "_Run_Lowlevel execution interval: %lld us (%.3f ms)", 
  //               time_diff, time_diff / 1000.0);
  // }
  this->last_execution_time_ = _start_time;
}

void StateRL::_Run_Lowlevel()
{
  // 使用系统时间进行频率控制（默认 5ms）
  while (this->threadRunning)
  {
    long long _start_time = getSystemTime();

    if (!this->stop_update_)
    {
      this->actuate[this->dof_mode]();
    }
    
    // 计算并打印两次执行之间的时间间隔
    // if (this->last_execution_time_ > 0)
    // {
    //   long long time_diff = _start_time - this->last_execution_time_;
    //   RCLCPP_INFO(logger_, "_Run_Lowlevel execution interval: %lld us (%.3f ms)", 
    //               time_diff, time_diff / 1000.0);
    // }
    this->last_execution_time_ = _start_time;
    
    // 使用系统时间进行频率控制（默认 5ms）
    absoluteWait(_start_time, (long long)(0.005 * 1000000));
  }
  this->threadRunning = false;
}

void StateRL::_P_actuate()
{
  // 从线程安全的副本中获取 robot_state
  RobotState robot_state;
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state = robot_state_copy_;
  }
  
  // 计算 PID 输出
  double local_torque[6] = {0};
  for (int i=0; i < 6; i++){
    if (i < static_cast<int>(robot_state.joints.size())) {
      // std::cout << robot_state.joints[i].name << std::endl;
      this->pid_controllers[i].target = this->desired_pos[i];
      // this->pid_controllers[i].target = 0;
      this->pid_controllers[i].current = robot_state.joints[i].position;
      this->pid_controllers[i].Adjust(0, robot_state.joints[i].velocity);
      local_torque[i] = this->pid_controllers[i].out;
      if(i>3){
        local_torque[i] += this->desired_pos[i];
        if(local_torque[i]>4.99){
          local_torque[i] = 4.99;
        }
        if(local_torque[i]<-4.99){
          local_torque[i] = -4.99;
        }
      }
    }
  }
  
  // 线程安全地更新 torque 数组
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    for (int i=0; i < 6; i++) {
      this->torque[i] = local_torque[i];
    }
  }
}

void StateRL::_V_actuate()
{
  // 从线程安全的副本中获取 robot_state
  RobotState robot_state;
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state = robot_state_copy_;
  }
  
  // 计算速度控制输出
  double local_torque[6] = {0};
  for (int i=0; i < 6; i++){
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
    for (int i=0; i < 6; i++) {
      this->torque[i] = local_torque[i];
    }
  }
}

void StateRL::_T_actuate()
{
  // 从线程安全的副本中获取 robot_state
  RobotState robot_state;
  {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state = robot_state_copy_;
  }
  
  // 计算力矩控制输出
  double local_torque[6] = {0};
  for (int i=0; i < 6; i++){
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
    for (int i=0; i < 6; i++) {
      this->torque[i] = local_torque[i];
    }
  }
}

}  // namespace robot_locomotion



