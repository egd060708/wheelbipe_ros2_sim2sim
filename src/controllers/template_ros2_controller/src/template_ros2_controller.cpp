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

#include "template_ros2_controller/template_ros2_controller.hpp"
// #include <template_ros2_controller/template_ros2_controller_parameters.hpp>
#include "utils/timeMarker.h"
#include <mutex>

#include "pluginlib/class_list_macros.hpp"
namespace robot_locomotion
{
TemplateRos2Controller::TemplateRos2Controller() {}

controller_interface::CallbackReturn TemplateRos2Controller::on_init()
{
  try {
    // 声名参数（通过参数文件定义接口的名称）
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    sensor_names_ = auto_declare<std::vector<std::string>>("sensors", sensor_names_);
    // 创建imu传感器对象，并初始化（使用第一个传感器的名字）
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
      semantic_components::IMUSensor(sensor_names_[0]));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during on_init stage with message: %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // param list init
  // 参数监听器初始化
  param_listener_ = std::make_shared<template_ros2_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  
  // Initialize time synchronization
  time_sync_enabled_ = false;  // params_.sync_time_with_webots;  // 暂时禁用
  last_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  
  // 初始化状态机（传递节点指针用于创建定时器）
  state_machine_ = std::make_unique<StateMachine>(get_node()->get_logger(), get_node());
  
  // 初始化 RL 推理器（如果模型路径已配置）
  if (!params_.rl_model_path.empty()) {
    if (state_machine_->initializeRLInference(
        params_.rl_model_path, params_.rl_inference_frequency)) {
      RCLCPP_INFO(get_node()->get_logger(), 
        "RL inference initialized in on_init with model: %s, frequency: %ld Hz",
        params_.rl_model_path.c_str(), static_cast<long>(params_.rl_inference_frequency));
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), 
        "Failed to initialize RL inference in on_init");
    }
  } else {
    RCLCPP_INFO(get_node()->get_logger(), 
      "RL model path not configured, RL inference will not be available");
  }
  
  // 设置调度配置
  state_machine_->setTimingConfig(
    params_.inference_mode,
    params_.lowlevel_mode,
    params_.inference_timer_clock_type,
    params_.lowlevel_timer_clock_type);
  state_machine_->setLowlevelFrequency(params_.rl_lowlevel_frequency);
  state_machine_->setInferenceFrequency(static_cast<double>(params_.rl_inference_frequency));
  
  // 设置关节参数
  if (!params_.joint_stiffness.empty() && 
      !params_.joint_damping.empty() && 
      !params_.joint_action_scale.empty() &&
      !params_.joint_output_max.empty() &&
      !params_.joint_output_min.empty() &&
      !params_.joint_bias.empty() &&
      !params_.default_dof_pos.empty()) {
    state_machine_->setJointParams(
      params_.joint_stiffness,
      params_.joint_damping,
      params_.joint_action_scale,
      params_.joint_output_max,
      params_.joint_output_min,
      params_.joint_bias,
      params_.default_dof_pos);
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  // 如果没有关节参数，将报错
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  // 对关节类型进行初始化
  for (std::string & joint_name : joint_names_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Get joint name : %s", joint_name.c_str());
    std::shared_ptr<Joint> joint = std::make_shared<Joint>();
    joint->name = joint_name;
    joints_.emplace_back(joint);
  }

  // 初始化机器人状态数据结构
  robot_state_.joints.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    robot_state_.joints[i].name = joint_names_[i];
  }

  // 初始化运动指令数据
  motion_cmd_received_ = false;
  latest_lin_vel_x_ = 0.0;
  latest_lin_vel_y_ = 0.0;
  latest_ang_vel_z_ = 0.0;
  latest_height_ = 0.2;  // 默认高度

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TemplateRos2Controller::command_interface_configuration() const
{
  // 获取下发指令接口名称
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : command_interface_types_) {
      conf_names.push_back(joint->name + "/" + interface_type);
    }
  }
  // INDIVIDUAL控制器需要单独列出每个接口的名称，ALL控制器需要访问所有可用的接口，NONE控制器不需要任何接口
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration TemplateRos2Controller::state_interface_configuration() const
{
  // 自定义状态接口
  // 关节接口
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : state_interface_types_)
      conf_names.push_back(joint->name + "/" + interface_type);
  }
  // imu接口
  for (auto name : imu_sensor_->get_state_interface_names()) conf_names.push_back(name);
  // 控制器需要单独列出每个接口的名称
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type TemplateRos2Controller::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // std::cout << period.nanoseconds() / 1000.0 << std::endl;
  // long long _start_time = getSystemTime();
  // static long long last_execution_time_ = getSystemTime();
  // // 计算并打印两次执行之间的时间间隔
  // if (last_execution_time_ > 0)
  // {
  //   long long time_diff = _start_time - last_execution_time_;
  //   std::cout << "controller execution interval: " << time_diff << "us (" << time_diff / 1000.0 <<" ms)" << std::endl; 
  // }
  // last_execution_time_ = _start_time;
  // 更新参数（如果发生变化）
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    // 更新时间/调度配置
    if (state_machine_) {
      state_machine_->setTimingConfig(
        params_.inference_mode,
        params_.lowlevel_mode,
        params_.inference_timer_clock_type,
        params_.lowlevel_timer_clock_type);
      state_machine_->setLowlevelFrequency(params_.rl_lowlevel_frequency);
      state_machine_->setInferenceFrequency(static_cast<double>(params_.rl_inference_frequency));
      // 更新关节参数
      if (!params_.joint_stiffness.empty() && 
          !params_.joint_damping.empty() && 
          !params_.joint_action_scale.empty() &&
          !params_.joint_output_max.empty() &&
          !params_.joint_output_min.empty() &&
          !params_.joint_bias.empty() &&
          !params_.default_dof_pos.empty()) {
        state_machine_->setJointParams(
          params_.joint_stiffness,
          params_.joint_damping,
          params_.joint_action_scale,
          params_.joint_output_max,
          params_.joint_output_min,
          params_.joint_bias,
          params_.default_dof_pos);
      }
    }
  }

  // 1. 更新机器人状态数据结构
  updateRobotState(time, period);

  // 2. 状态机调度：根据当前机器人状态进行状态转换和输出计算
  if (state_machine_) {
    state_machine_->update(robot_state_, time, period);
    
    // 3. 从 RobotState 获取输出力矩并应用到关节
    for (size_t i = 0; i < joints_.size() && i < robot_state_.joints.size(); ++i) {
      joints_[i]->effort_command_handle->get().set_value(robot_state_.joints[i].output_torque);
    }
    
    // 如果关节数量不匹配，使用默认逻辑
    // if (robot_state_.joints.size() != joints_.size()) {
    //   // 默认逻辑：保持原有行为（用于兼容）
    //   for (std::shared_ptr<Joint> joint : joints_) {
    //     if (joint->name == "left_spring2_joint" || joint->name == "right_spring2_joint") {
    //       joint->effort_command_handle->get().set_value(240);
    //     } else {
    //       joint->effort_command_handle->get().set_value(0);
    //     }
    //   }
    // }
  } else {
    // 如果状态机未初始化，使用默认逻辑
    for (std::shared_ptr<Joint> joint : joints_) {
      joint->effort_command_handle->get().set_value(0);
      if (joint->name == "left_spring2_joint" || joint->name == "right_spring2_joint") {
        joint->effort_command_handle->get().set_value(240);
      }
    }
  }

  return controller_interface::return_type::OK;
}

void TemplateRos2Controller::updateRobotState(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // 更新关节状态
  for (size_t i = 0; i < joints_.size() && i < robot_state_.joints.size(); ++i) {
    robot_state_.joints[i].position = joints_[i]->position_handle->get().get_value();
    robot_state_.joints[i].velocity = joints_[i]->velocity_handle->get().get_value();
    robot_state_.joints[i].effort = joints_[i]->effort_handle->get().get_value();
  }

  // 更新IMU传感器状态
  std::array<double, 3> accl = imu_sensor_->get_linear_acceleration();
  std::array<double, 3> gyro = imu_sensor_->get_angular_velocity();
  std::array<double, 4> quat = imu_sensor_->get_orientation();
  
  robot_state_.imu.linear_acceleration[0] = accl[0];
  robot_state_.imu.linear_acceleration[1] = accl[1];
  robot_state_.imu.linear_acceleration[2] = accl[2];
  
  robot_state_.imu.angular_velocity[0] = gyro[0];
  robot_state_.imu.angular_velocity[1] = gyro[1];
  robot_state_.imu.angular_velocity[2] = gyro[2];
  
  robot_state_.imu.orientation[0] = quat[3];
  robot_state_.imu.orientation[1] = quat[0];
  robot_state_.imu.orientation[2] = quat[1];
  robot_state_.imu.orientation[3] = quat[2];

  // 更新时间信息
  robot_state_.timestamp = time.seconds();
  robot_state_.period = period.seconds();

  // 更新运动指令（从订阅的话题中获取）
  {
    std::lock_guard<std::mutex> lock(motion_cmd_mutex_);
    robot_state_.command.cmd_vel[0] = latest_lin_vel_x_;
    robot_state_.command.cmd_vel[1] = latest_lin_vel_y_;
    robot_state_.command.cmd_vel[2] = latest_ang_vel_z_;
    robot_state_.command.cmd_height = latest_height_;
    
    // 定期打印命令值（每100次更新打印一次，约0.2秒一次）
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0) {
      RCLCPP_DEBUG(get_node()->get_logger(),
        "Current command: lin_vel_x=%.3f, lin_vel_y=%.3f, ang_vel_z=%.3f, height=%.3f, received=%d",
        latest_lin_vel_x_, latest_lin_vel_y_, latest_ang_vel_z_, latest_height_, motion_cmd_received_);
    }
  }

  // 计算广义状态
  robot_state_.run();
}

void TemplateRos2Controller::motionCommandCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(motion_cmd_mutex_);
  latest_lin_vel_x_ = msg->linear.x;
  latest_lin_vel_y_ = msg->linear.y;
  latest_ang_vel_z_ = msg->angular.z;
  motion_cmd_received_ = true;
  RCLCPP_INFO(get_node()->get_logger(), 
    "Received motion command: lin_vel_x=%.3f, lin_vel_y=%.3f, ang_vel_z=%.3f",
    msg->linear.x, msg->linear.y, msg->angular.z);
}

void TemplateRos2Controller::heightCommandCallback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(motion_cmd_mutex_);
  latest_height_ = msg->data;
  RCLCPP_INFO(get_node()->get_logger(), "Received height command: %.3f", msg->data);
}

// 从失能状态到激活状态
controller_interface::CallbackReturn TemplateRos2Controller::on_activate(const rclcpp_lifecycle::State &)
{
  // 创建运动指令话题订阅者
  std::string motion_topic = "motion_command";
  std::string height_topic = "height_command";
  
  motion_cmd_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    motion_topic, 10,
    std::bind(&TemplateRos2Controller::motionCommandCallback, this, std::placeholders::_1));
  height_cmd_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    height_topic, 10,
    std::bind(&TemplateRos2Controller::heightCommandCallback, this, std::placeholders::_1));
  
  // 获取节点的完整话题名称（包括命名空间）
  std::string node_namespace = get_node()->get_namespace();
  RCLCPP_INFO(get_node()->get_logger(), 
    "Motion command subscribers created. Node namespace: '%s'", node_namespace.c_str());
  RCLCPP_INFO(get_node()->get_logger(), 
    "Subscribing to topics: '%s' (full: '%s'), '%s' (full: '%s')",
    motion_topic.c_str(), motion_cmd_subscriber_->get_topic_name(),
    height_topic.c_str(), height_cmd_subscriber_->get_topic_name());

  // 查找并绑定关节命令接口
  RCLCPP_INFO(get_node()->get_logger(), "on_activate");
  for (std::shared_ptr<Joint> joint : joints_) {
    // Position command
    // find_if在范围内查找第一个满足特定条件的元素，begin和end是搜索范围，predicate一个函数或者lambda表达式，定义搜索条件
    // 在上面初始化中配置了INDIVIDUAL后，所有的接口在此处一一绑定
    // end不指向范围内的元素，而是指向范围的末尾（判断空）
    const auto position_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint](const auto & interface)
      {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    // 解引用迭代器，获取对象的真实引用，并通过reference_wrapper安全存储
    joint->position_command_handle = std::ref(*position_command_handle);

    // Velocity command
    const auto velocity_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint](const auto & interface)
      {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_command_handle = std::ref(*velocity_command_handle);

    // Effort command
    const auto effort_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain effort command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_command_handle = std::ref(*effort_command_handle);
    // Position state
    const auto position_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_handle = std::ref(*position_handle);
    // Velocity state
    const auto velocity_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_handle = std::ref(*velocity_handle);
    // Effort state
    const auto effort_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_handle = std::ref(*effort_handle);
  }
  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_deactivate(const rclcpp_lifecycle::State &)
{
  // 清零力矩命令
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate ");
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->effort_command_handle->get().set_value(0);
  }
  imu_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_cleanup ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_error ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_shutdown ");
  return controller_interface::CallbackReturn::SUCCESS;
}


TemplateRos2Controller::~TemplateRos2Controller()
{
}

}  // namespace robot_locomotion

#include "class_loader/register_macro.hpp"

PLUGINLIB_EXPORT_CLASS(robot_locomotion::TemplateRos2Controller, controller_interface::ControllerInterface)