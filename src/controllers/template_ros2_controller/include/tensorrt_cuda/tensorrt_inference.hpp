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

#ifndef TEMPLATE_ROS2_CONTROLLER__TENSORRT_INFERENCE_HPP_
#define TEMPLATE_ROS2_CONTROLLER__TENSORRT_INFERENCE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// TensorRT includes (conditional compilation)
#ifdef TENSORRT_AVAILABLE
#include <NvInfer.h>
#include <cuda_runtime.h>
#endif

namespace robot_locomotion
{

  // TensorRT 推理引擎类
  class TensorRTInference
  {
  public:
    TensorRTInference(rclcpp::Logger logger);
    ~TensorRTInference();

    // 初始化：加载 TensorRT engine 模型
    bool initialize(const std::string &engine_model_path, int inference_frequency_hz);

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

    // 设置是否使用 period 进行频率计算（需要传递节点指针用于创建定时器）
    void setUsePeriodTiming(bool use_period, rclcpp_lifecycle::LifecycleNode::SharedPtr node = nullptr);

    // 推理回调函数（用于定时器）
    void inferenceCallback();

  private:
    // TensorRT 相关
    bool loadEngineFromFile(const std::string &engine_model_path);
    bool createExecutionContext();
    void destroyEngine();

    // CUDA 相关
    bool allocateBuffers();
    void freeBuffers();

    // 推理线程函数
    void inferenceThread();

    // Logger
    rclcpp::Logger logger_;

    // 模型信息
    std::string engine_model_path_;
    int inference_frequency_hz_;
    std::chrono::milliseconds inference_period_ms_;

    // TensorRT 对象
#ifdef TENSORRT_AVAILABLE
    nvinfer1::IRuntime *runtime_;
    nvinfer1::ICudaEngine *engine_;
    nvinfer1::IExecutionContext *context_;
#else
    void *runtime_;
    void *engine_;
    void *context_;
#endif

    // CUDA 缓冲区
    void *input_buffer_;
    void *output_buffer_;
    size_t input_size_;
    size_t output_size_;
    int input_binding_index_;
    int output_binding_index_;

    // TensorRT 10: 使用张量名称而不是绑定索引
    std::string input_tensor_name_;
    std::string output_tensor_name_;

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

    // CUDA 流
#ifdef TENSORRT_AVAILABLE
    cudaStream_t cuda_stream_;
#else
    void *cuda_stream_;
#endif

    // 用于记录上次执行时间（用于时间间隔统计）
    std::chrono::steady_clock::time_point last_execution_time_;

    // 是否使用 period 进行频率计算
    std::atomic<bool> use_period_timing_;

    // ROS2 定时器（当使用 ROS2 时间时）
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr ros_timer_;
    int64_t period_nanoseconds_;
  };
} // namespace robot_locomotion

#endif // TEMPLATE_ROS2_CONTROLLER__TENSORRT_INFERENCE_HPP_
