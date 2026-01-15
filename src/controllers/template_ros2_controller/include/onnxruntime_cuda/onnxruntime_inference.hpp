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

#ifndef TEMPLATE_ROS2_CONTROLLER__ONNXRUNTIME_INFERENCE_HPP_
#define TEMPLATE_ROS2_CONTROLLER__ONNXRUNTIME_INFERENCE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rcl/time.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// ONNX Runtime includes (conditional compilation)
#ifdef ONNXRUNTIME_AVAILABLE
#include <onnxruntime_cxx_api.h>
#include <cuda_runtime.h>
#endif

namespace robot_locomotion
{

// ONNX Runtime 推理引擎类（接口与 TensorRTInference 保持一致）
class ONNXRuntimeInference
{
public:
  ONNXRuntimeInference(rclcpp::Logger logger);
  ~ONNXRuntimeInference();

  // 初始化：加载 ONNX 模型
  bool initialize(const std::string &model_path, int inference_frequency_hz, bool use_cuda = false);

  // （可选）在创建 session 之前手动指定张量名称。
  // 传入空字符串表示该项仍由自动探测逻辑决定。
  void setTensorNames(const std::string &input_name,
                      const std::string &actions_name,
                      const std::string &lin_vel_name,
                      const std::string &height_name);

  // 启动推理线程
  void start();

  // 停止推理线程
  void stop();

  // 设置输入数据（线程安全）
  void setInput(const std::vector<float> &input_data);

  // 获取输出数据（线程安全）
  bool getOutput(std::vector<float> &output_data);

  // 检查是否已初始化
  bool isInitialized() const { return initialized_; }

  // 检查推理线程是否运行
  bool isRunning() const { return running_; }

  // 设置调度模式：0=线程(系统时间)，1=ROS2定时器，2=内联
  // 可选择定时器的时钟类型
  void setMode(int mode, rcl_clock_type_t clock_type,
               rclcpp_lifecycle::LifecycleNode::SharedPtr node = nullptr);

  // 推理回调函数（用于定时器或内联调用）
  void inferenceCallback();

private:
  // ONNX Runtime 相关
  bool loadModelFromFile(const std::string &model_path);
  bool createSession();
  void destroySession();

  // CUDA 相关（如果使用 GPU）
  bool allocateBuffers();
  void freeBuffers();

  // 推理线程函数
  void inferenceThread();

  // Logger
  rclcpp::Logger logger_;

  // 模型信息
  std::string model_path_;
  int inference_frequency_hz_;
  std::chrono::milliseconds inference_period_ms_;

  // ONNX Runtime 对象
#ifdef ONNXRUNTIME_AVAILABLE
  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  Ort::SessionOptions session_options_;
  Ort::MemoryInfo memory_info_cpu_{nullptr};  // 初始化为 nullptr，在 createSession 中初始化
  Ort::MemoryInfo memory_info_gpu_{nullptr};  // 初始化为 nullptr，在 createSession 中初始化
  bool use_cuda_;
  int cuda_device_id_;
#else
  void *env_;
  void *session_;
  bool use_cuda_;
  int cuda_device_id_;
#endif

  // 输入输出张量名称
  std::string input_tensor_name_;
  std::string output_actions_name_;
  std::string output_lin_vel_name_;
  std::string output_height_name_;

  // 输入输出形状信息
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_actions_shape_;
  std::vector<int64_t> output_lin_vel_shape_;
  std::vector<int64_t> output_height_shape_;

  // 输入输出大小（元素个数）
  size_t input_size_;
  size_t output_actions_size_;
  size_t output_lin_vel_size_;
  size_t output_height_size_;

  // CUDA 缓冲区（如果使用 GPU）
  void *input_buffer_gpu_;
  void *output_actions_buffer_gpu_;
  void *output_lin_vel_buffer_gpu_;
  void *output_height_buffer_gpu_;

  // 线程控制
  std::thread inference_thread_;
  std::atomic<bool> running_;
  std::atomic<bool> initialized_;
  std::mutex data_mutex_;
  std::condition_variable data_ready_cv_;
  std::condition_variable inference_done_cv_;

  // 数据缓冲区
  std::vector<float> input_data_;
  std::vector<float> output_data_;
  std::atomic<bool> input_ready_;
  std::atomic<bool> output_ready_;
  std::atomic<bool> new_input_available_;

  // CUDA 流（如果使用 GPU）
#ifdef ONNXRUNTIME_AVAILABLE
  cudaStream_t cuda_stream_;
#else
  void *cuda_stream_;
#endif

  // 用于记录上次执行时间（用于时间间隔统计）
  std::chrono::steady_clock::time_point last_execution_time_;

  // 调度模式与时钟
  std::atomic<int> mode_;
  rcl_clock_type_t timer_clock_type_;

  // ROS2 定时器（当使用 ROS2 时间/其他时钟时）
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr ros_timer_;
  int64_t period_nanoseconds_;
};

} // namespace robot_locomotion

#endif // TEMPLATE_ROS2_CONTROLLER__ONNXRUNTIME_INFERENCE_HPP_

