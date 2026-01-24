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

#include "onnxruntime_cuda/onnxruntime_inference.hpp"
#include <fstream>
#include <iostream>
#include <cstring>
#include "rclcpp/create_timer.hpp"
#include "rcl/time.h"

#ifdef ONNXRUNTIME_AVAILABLE

namespace robot_locomotion
{

ONNXRuntimeInference::ONNXRuntimeInference(rclcpp::Logger logger)
  : logger_(logger)
  , inference_frequency_hz_(100)
  , inference_period_ms_(10)
  , env_(nullptr)
  , session_(nullptr)
  , use_cuda_(false)  // 默认使用 CPU，避免 CUBLAS 错误
  , cuda_device_id_(0)
  , input_tensor_name_()
  , output_actions_name_()
  , output_lin_vel_name_()
  , output_height_name_()
  , input_size_(0)
  , output_actions_size_(0)
  , output_lin_vel_size_(0)
  , output_height_size_(0)
  , input_buffer_gpu_(nullptr)
  , output_actions_buffer_gpu_(nullptr)
  , output_lin_vel_buffer_gpu_(nullptr)
  , output_height_buffer_gpu_(nullptr)
  , running_(false)
  , initialized_(false)
  , input_ready_(false)
  , output_ready_(false)
  , new_input_available_(false)
  , cuda_stream_(nullptr)
  , last_execution_time_{}
  , mode_(0)
  , timer_clock_type_(RCL_ROS_TIME)
  , node_(nullptr)
  , period_nanoseconds_(0)
{
}

ONNXRuntimeInference::~ONNXRuntimeInference()
{
  stop();
  destroySession();
  freeBuffers();
}

void ONNXRuntimeInference::setTensorNames(const std::string& input_name,
                                          const std::string& actions_name,
                                          const std::string& lin_vel_name,
                                          const std::string& height_name)
{
  // 仅保存"期望名称"，具体是否存在由 createSession 中进行检查和回退
  if (!input_name.empty()) {
    input_tensor_name_ = input_name;
  }
  if (!actions_name.empty()) {
    output_actions_name_ = actions_name;
  }
  if (!lin_vel_name.empty()) {
    output_lin_vel_name_ = lin_vel_name;
  }
  if (!height_name.empty()) {
    output_height_name_ = height_name;
  }
}

bool ONNXRuntimeInference::initialize(const std::string& model_path, int inference_frequency_hz, bool use_cuda)
{
  if (initialized_) {
    RCLCPP_WARN(logger_, "ONNX Runtime inference already initialized");
    return true;
  }

  model_path_ = model_path;
  inference_frequency_hz_ = inference_frequency_hz;
  inference_period_ms_ = std::chrono::milliseconds(1000 / inference_frequency_hz_);
  period_nanoseconds_ = (1000000000LL / inference_frequency_hz_);
  use_cuda_ = use_cuda;  // 设置是否使用 CUDA

  // 检查文件是否存在
  std::ifstream file(model_path, std::ios::binary);
  if (!file.good()) {
    RCLCPP_ERROR(logger_, "ONNX model file not found: %s", model_path.c_str());
    return false;
  }
  file.close();

  // 加载 ONNX 模型
  if (!loadModelFromFile(model_path)) {
    RCLCPP_ERROR(logger_, "Failed to load ONNX model from file");
    return false;
  }

  // 创建 session
  if (!createSession()) {
    RCLCPP_ERROR(logger_, "Failed to create ONNX Runtime session");
    return false;
  }

  // 分配缓冲区
  if (!allocateBuffers()) {
    RCLCPP_ERROR(logger_, "Failed to allocate buffers");
    return false;
  }

  // 创建 CUDA 流（如果使用 GPU）
  if (use_cuda_) {
    if (cudaStreamCreate(&cuda_stream_) != cudaSuccess) {
      RCLCPP_ERROR(logger_, "Failed to create CUDA stream");
      return false;
    }
  }

  initialized_ = true;
  RCLCPP_INFO(logger_, "ONNX Runtime inference initialized successfully. Model: %s, Frequency: %d Hz, Device: %s",
    model_path.c_str(), inference_frequency_hz_, use_cuda_ ? "CUDA" : "CPU");
  
  return true;
}

bool ONNXRuntimeInference::loadModelFromFile(const std::string& model_path)
{
  (void)model_path;  // 参数未使用，但保留以保持接口一致性
  // 创建 ONNX Runtime 环境
  env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntimeInference");
  
  RCLCPP_INFO(logger_, "ONNX Runtime environment created");
  return true;
}

bool ONNXRuntimeInference::createSession()
{
  if (!env_) {
    RCLCPP_ERROR(logger_, "Environment not created, cannot create session");
    return false;
  }

  // 配置 session 选项
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
  session_options_.SetIntraOpNumThreads(1);
  session_options_.SetInterOpNumThreads(1);

  // 尝试使用 CUDA Execution Provider
  if (use_cuda_) {
    try {
      OrtCUDAProviderOptions cuda_options{};
      cuda_options.device_id = cuda_device_id_;
      cuda_options.arena_extend_strategy = 0;
      cuda_options.gpu_mem_limit = static_cast<size_t>(2) * 1024 * 1024 * 1024;  // 2GB
      cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
      cuda_options.do_copy_in_default_stream = 1;
      
      session_options_.AppendExecutionProvider_CUDA(cuda_options);
      RCLCPP_INFO(logger_, "CUDA Execution Provider enabled (device_id=%d)", cuda_device_id_);
      
      // 创建 GPU memory info（需要4个参数：name, allocator_type, device_id, mem_type）
      memory_info_gpu_ = Ort::MemoryInfo("Cuda", OrtDeviceAllocator, cuda_device_id_, OrtMemTypeDefault);
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Failed to enable CUDA Execution Provider: %s. Falling back to CPU.", e.what());
      use_cuda_ = false;
    }
  }

  // 创建 CPU memory info（总是需要）
  memory_info_cpu_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  // 创建 session
  try {
    session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to create ONNX Runtime session: %s", e.what());
    return false;
  }

  // 获取输入输出张量信息
  size_t num_input_nodes = session_->GetInputCount();
  size_t num_output_nodes = session_->GetOutputCount();

  RCLCPP_INFO(logger_, "ONNX model loaded. Inputs: %zu, Outputs: %zu", num_input_nodes, num_output_nodes);

  // 查找输入张量（ONNX Runtime 1.20.x 使用新的 API）
  std::string first_input_name;
  Ort::AllocatorWithDefaultOptions allocator;
  for (size_t i = 0; i < num_input_nodes; ++i) {
    auto input_name = session_->GetInputNameAllocated(i, allocator);
    if (!input_name) continue;

    std::string name(input_name.get());

    if (first_input_name.empty()) {
      first_input_name = name;
    }

    if (input_tensor_name_.empty() || name == input_tensor_name_) {
      input_tensor_name_ = name;
      auto type_info = session_->GetInputTypeInfo(i);
      auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
      input_shape_ = tensor_info.GetShape();
      
      input_size_ = 1;
      for (auto dim : input_shape_) {
        if (dim > 0) {
          input_size_ *= dim;
        }
      }
      
      RCLCPP_INFO(logger_, "Found input tensor: %s, shape: [%s], size: %zu",
        name.c_str(), 
        [&]() {
          std::string shape_str;
          for (size_t j = 0; j < input_shape_.size(); ++j) {
            if (j > 0) shape_str += ", ";
            shape_str += std::to_string(input_shape_[j]);
          }
          return shape_str;
        }().c_str(),
        input_size_);
      break;
    }
  }

  if (input_tensor_name_.empty()) {
    if (!first_input_name.empty()) {
      input_tensor_name_ = first_input_name;
      RCLCPP_WARN(logger_, "Input tensor '%s' not found, using first input: %s",
        input_tensor_name_.c_str(), first_input_name.c_str());
    } else {
      RCLCPP_ERROR(logger_, "No input tensors found in the model");
      return false;
    }
  }

  // 查找输出张量（ONNX Runtime 1.20.x 使用新的 API）
  std::vector<std::string> output_names;
  for (size_t i = 0; i < num_output_nodes; ++i) {
    auto output_name = session_->GetOutputNameAllocated(i, allocator);
    if (!output_name) continue;

    std::string name(output_name.get());
    output_names.push_back(name);

    auto type_info = session_->GetOutputTypeInfo(i);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    auto shape = tensor_info.GetShape();

    size_t size = 1;
    for (auto dim : shape) {
      if (dim > 0) {
        size *= dim;
      }
    }

    if (name == output_actions_name_ || (output_actions_name_.empty() && i == 0)) {
      output_actions_name_ = name;
      output_actions_shape_ = shape;
      output_actions_size_ = size;
      RCLCPP_INFO(logger_, "Found actions output tensor: %s, shape: [%s], size: %zu",
        name.c_str(),
        [&]() {
          std::string shape_str;
          for (size_t j = 0; j < shape.size(); ++j) {
            if (j > 0) shape_str += ", ";
            shape_str += std::to_string(shape[j]);
          }
          return shape_str;
        }().c_str(),
        size);
    } else if (name == output_lin_vel_name_) {
      output_lin_vel_name_ = name;
      output_lin_vel_shape_ = shape;
      output_lin_vel_size_ = size;
      RCLCPP_INFO(logger_, "Found lin_vel output tensor: %s, shape: [%s], size: %zu",
        name.c_str(),
        [&]() {
          std::string shape_str;
          for (size_t j = 0; j < shape.size(); ++j) {
            if (j > 0) shape_str += ", ";
            shape_str += std::to_string(shape[j]);
          }
          return shape_str;
        }().c_str(),
        size);
    } else if (name == output_height_name_) {
      output_height_name_ = name;
      output_height_shape_ = shape;
      output_height_size_ = size;
      RCLCPP_INFO(logger_, "Found height output tensor: %s, shape: [%s], size: %zu",
        name.c_str(),
        [&]() {
          std::string shape_str;
          for (size_t j = 0; j < shape.size(); ++j) {
            if (j > 0) shape_str += ", ";
            shape_str += std::to_string(shape[j]);
          }
          return shape_str;
        }().c_str(),
        size);
    }
  }

  // 如果没有找到 actions，使用第一个输出
  if (output_actions_name_.empty() && !output_names.empty()) {
    output_actions_name_ = output_names[0];
    RCLCPP_WARN(logger_, "Actions output tensor not found, using first output: %s",
      output_actions_name_.c_str());
  }

  if (output_actions_name_.empty()) {
    RCLCPP_ERROR(logger_, "No usable output tensor found (expected at least 'actions')");
    return false;
  }

  // 初始化数据缓冲区
  input_data_.resize(input_size_);
  output_data_.resize(output_actions_size_);

  RCLCPP_INFO(logger_, "ONNX Runtime session created successfully");
  return true;
}

bool ONNXRuntimeInference::allocateBuffers()
{
  if (use_cuda_ && cuda_stream_) {
    // 分配 GPU 缓冲区
    size_t input_bytes = input_size_ * sizeof(float);
    size_t output_actions_bytes = output_actions_size_ * sizeof(float);
    size_t output_lin_vel_bytes = output_lin_vel_size_ * sizeof(float);
    size_t output_height_bytes = output_height_size_ * sizeof(float);

    if (cudaMalloc(&input_buffer_gpu_, input_bytes) != cudaSuccess) {
      RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for input");
      return false;
    }

    if (cudaMalloc(&output_actions_buffer_gpu_, output_actions_bytes) != cudaSuccess) {
      RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for actions output");
      cudaFree(input_buffer_gpu_);
      return false;
    }

    if (output_lin_vel_size_ > 0) {
      if (cudaMalloc(&output_lin_vel_buffer_gpu_, output_lin_vel_bytes) != cudaSuccess) {
        RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for lin_vel output");
        cudaFree(input_buffer_gpu_);
        cudaFree(output_actions_buffer_gpu_);
        return false;
      }
    }

    if (output_height_size_ > 0) {
      if (cudaMalloc(&output_height_buffer_gpu_, output_height_bytes) != cudaSuccess) {
        RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for height output");
        cudaFree(input_buffer_gpu_);
        cudaFree(output_actions_buffer_gpu_);
        if (output_lin_vel_buffer_gpu_) {
          cudaFree(output_lin_vel_buffer_gpu_);
        }
        return false;
      }
    }

    RCLCPP_INFO(logger_, "CUDA buffers allocated for ONNX Runtime");
  }

  return true;
}

void ONNXRuntimeInference::freeBuffers()
{
  if (use_cuda_ && cuda_stream_) {
    if (input_buffer_gpu_) {
      cudaFree(input_buffer_gpu_);
      input_buffer_gpu_ = nullptr;
    }
    if (output_actions_buffer_gpu_) {
      cudaFree(output_actions_buffer_gpu_);
      output_actions_buffer_gpu_ = nullptr;
    }
    if (output_lin_vel_buffer_gpu_) {
      cudaFree(output_lin_vel_buffer_gpu_);
      output_lin_vel_buffer_gpu_ = nullptr;
    }
    if (output_height_buffer_gpu_) {
      cudaFree(output_height_buffer_gpu_);
      output_height_buffer_gpu_ = nullptr;
    }
    cudaStreamDestroy(cuda_stream_);
    cuda_stream_ = nullptr;
  }
}

void ONNXRuntimeInference::destroySession()
{
  session_.reset();
  env_.reset();
}

void ONNXRuntimeInference::start()
{
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "Cannot start inference: not initialized");
    return;
  }

  if (running_) {
    RCLCPP_WARN(logger_, "Inference thread already running");
    return;
  }

  running_ = true;
  
  if (mode_ == 2) {
    // 内联模式，不启动线程/定时器
    RCLCPP_INFO(logger_, "ONNX Runtime inference inline mode (no thread/timer)");
    return;
  }

  if (mode_ == 1) {
    if (!node_) {
      RCLCPP_WARN(logger_, "No node available, fallback to thread mode");
      mode_ = 0;
    } else if (period_nanoseconds_ > 0) {
      auto clock = std::make_shared<rclcpp::Clock>(timer_clock_type_);
      ros_timer_ = rclcpp::create_timer(
        node_->get_node_base_interface(),
        node_->get_node_timers_interface(),
        clock,
        rclcpp::Duration::from_nanoseconds(period_nanoseconds_),
        std::bind(&ONNXRuntimeInference::inferenceCallback, this),
        nullptr);
      RCLCPP_INFO(logger_, "ONNX Runtime inference started with ROS2 timer (mode=1, clock=%d, period=%ld ns)",
                  static_cast<int>(timer_clock_type_), period_nanoseconds_);
      return;
    } else {
      RCLCPP_WARN(logger_, "Period not set, fallback to thread mode");
      mode_ = 0;
    }
  }

  // 默认：线程+系统时间
  inference_thread_ = std::thread(&ONNXRuntimeInference::inferenceThread, this);
  RCLCPP_INFO(logger_, "ONNX Runtime inference thread started (mode=0 system time)");
}

void ONNXRuntimeInference::stop()
{
  if (!running_) {
    return;
  }

  running_ = false;
  data_ready_cv_.notify_all();

  // 停止 ROS2 定时器
  if (ros_timer_) {
    ros_timer_->cancel();
    ros_timer_.reset();
  }

  // 停止独立线程
  if (inference_thread_.joinable()) {
    inference_thread_.join();
  }

  RCLCPP_INFO(logger_, "ONNX Runtime inference stopped");
}

void ONNXRuntimeInference::setInput(const std::vector<float>& input_data)
{
  if (!initialized_) {
    return;
  }

  std::unique_lock<std::mutex> lock(data_mutex_);
  
  if (input_data.size() != input_data_.size()) {
    static rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(logger_, clock, 1000,
      "Input size mismatch: expected %zu, got %zu", input_data_.size(), input_data.size());
    return;
  }

  input_data_ = input_data;
  new_input_available_ = true;
  input_ready_ = true;

  if (mode_ == 2) {
    // 内联执行推理
    lock.unlock();
    inferenceCallback();
  } else {
    data_ready_cv_.notify_one();
  }
}

bool ONNXRuntimeInference::getOutput(std::vector<float>& output_data)
{
  if (!initialized_ || !output_ready_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (output_ready_) {
    output_data = output_data_;
    return true;
  }

  return false;
}

void ONNXRuntimeInference::setMode(int mode, rcl_clock_type_t clock_type,
                                    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  mode_ = mode;
  timer_clock_type_ = clock_type;
  node_ = node;
}

void ONNXRuntimeInference::inferenceCallback()
{
  // ROS2 定时器回调函数
  auto current_time = std::chrono::steady_clock::now();
  last_execution_time_ = current_time;

  // 执行推理
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (new_input_available_ && input_ready_) {
    try {
      // 如果使用 GPU，先拷贝数据到 GPU
      if (use_cuda_) {
        cudaMemcpyAsync(input_buffer_gpu_, input_data_.data(), input_size_ * sizeof(float),
                       cudaMemcpyHostToDevice, cuda_stream_);
        cudaStreamSynchronize(cuda_stream_);
      }

      // 准备输入张量（在 GPU 数据拷贝之后）
      // 如果 shape 中有动态维度（-1），需要根据实际数据大小推断
      std::vector<int64_t> actual_shape = input_shape_;
      size_t actual_input_size = input_data_.size();
      
      // 检查是否有动态维度
      bool has_dynamic_dim = false;
      for (auto& dim : actual_shape) {
        if (dim == -1) {
          has_dynamic_dim = true;
          break;
        }
      }
      
      // 如果有动态维度，根据实际数据大小推断
      if (has_dynamic_dim) {
        size_t fixed_size = 1;
        int dynamic_dim_index = -1;
        for (size_t i = 0; i < actual_shape.size(); ++i) {
          if (actual_shape[i] > 0) {
            fixed_size *= actual_shape[i];
          } else if (actual_shape[i] == -1) {
            dynamic_dim_index = i;
          }
        }
        if (dynamic_dim_index >= 0 && fixed_size > 0) {
          actual_shape[dynamic_dim_index] = actual_input_size / fixed_size;
          RCLCPP_DEBUG(logger_, "Inferred dynamic dimension %d: %ld (input_size=%zu, fixed_size=%zu)",
                      dynamic_dim_index, actual_shape[dynamic_dim_index], actual_input_size, fixed_size);
        }
      }
      
      // 验证实际输入大小与 shape 匹配
      size_t shape_size = 1;
      for (auto dim : actual_shape) {
        shape_size *= dim;
      }
      if (shape_size != actual_input_size) {
        RCLCPP_ERROR(logger_, "Input size mismatch: shape size=%zu, actual size=%zu. Shape: [%s]",
                    shape_size, actual_input_size,
                    [&]() {
                      std::string shape_str;
                      for (size_t j = 0; j < actual_shape.size(); ++j) {
                        if (j > 0) shape_str += ", ";
                        shape_str += std::to_string(actual_shape[j]);
                      }
                      return shape_str;
                    }().c_str());
        new_input_available_ = false;
        return;
      }

      Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        use_cuda_ ? memory_info_gpu_ : memory_info_cpu_,
        use_cuda_ ? static_cast<float*>(input_buffer_gpu_) : input_data_.data(),
        actual_input_size,
        actual_shape.data(),
        actual_shape.size());

      // 准备输出张量名称
      std::vector<const char*> output_names;
      if (!output_actions_name_.empty()) {
        output_names.push_back(output_actions_name_.c_str());
      }
      if (!output_lin_vel_name_.empty()) {
        output_names.push_back(output_lin_vel_name_.c_str());
      }
      if (!output_height_name_.empty()) {
        output_names.push_back(output_height_name_.c_str());
      }

      // 执行推理（ONNX Runtime 会自动创建输出张量）
      const char* input_names[] = {input_tensor_name_.c_str()};
      auto output_values = session_->Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_tensor,
        1,
        output_names.data(),
        output_names.size());

      // 拷贝输出数据回主机（仅 actions）
      if (!output_values.empty() && !output_actions_name_.empty()) {
        float* output_ptr = output_values[0].GetTensorMutableData<float>();
        
        if (use_cuda_) {
          // 如果使用 GPU，需要从 GPU 拷贝回 CPU
          cudaMemcpyAsync(output_data_.data(), output_ptr, 
                         output_actions_size_ * sizeof(float),
                         cudaMemcpyDeviceToHost, cuda_stream_);
          cudaStreamSynchronize(cuda_stream_);
        } else {
          // CPU 模式直接拷贝
          std::memcpy(output_data_.data(), output_ptr, 
                     output_actions_size_ * sizeof(float));
        }
      }

      output_ready_ = true;
      new_input_available_ = false;
      inference_done_cv_.notify_one();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "ONNX Runtime inference failed: %s", e.what());
      new_input_available_ = false;
    }
  }
}

void ONNXRuntimeInference::inferenceThread()
{
  RCLCPP_INFO(logger_, "ONNX Runtime inference thread started");

  auto next_inference_time = std::chrono::steady_clock::now();
  last_execution_time_ = std::chrono::steady_clock::time_point{};

  while (running_) {
    auto current_time = std::chrono::steady_clock::now();
    last_execution_time_ = current_time;
    
    // 等待输入数据或停止信号
    std::unique_lock<std::mutex> lock(data_mutex_);
    data_ready_cv_.wait(lock, [this] { return new_input_available_ || !running_; });

    if (!running_) {
      break;
    }

    if (new_input_available_ && input_ready_) {
      lock.unlock();
      inferenceCallback();
      lock.lock();
    }

    lock.unlock();

    // 使用系统时间进行频率控制（仅在不使用 ROS2 定时器时）
    next_inference_time += inference_period_ms_;
    std::this_thread::sleep_until(next_inference_time);
  }

  RCLCPP_INFO(logger_, "ONNX Runtime inference thread stopped");
}

}  // namespace robot_locomotion

#else  // ONNXRUNTIME_AVAILABLE not defined

// Stub implementation when ONNX Runtime is not available
namespace robot_locomotion
{

ONNXRuntimeInference::ONNXRuntimeInference(rclcpp::Logger logger)
  : logger_(logger)
  , inference_frequency_hz_(100)
  , inference_period_ms_(10)
  , env_(nullptr)
  , session_(nullptr)
  , use_cuda_(false)
  , cuda_device_id_(0)
  , running_(false)
  , initialized_(false)
  , input_ready_(false)
  , output_ready_(false)
  , new_input_available_(false)
  , cuda_stream_(nullptr)
  , last_execution_time_{}
  , mode_(0)
  , timer_clock_type_(RCL_ROS_TIME)
  , node_(nullptr)
  , period_nanoseconds_(0)
{
  RCLCPP_WARN(logger_, "ONNX Runtime not available. RL inference will not work.");
}

ONNXRuntimeInference::~ONNXRuntimeInference()
{
  stop();
}

bool ONNXRuntimeInference::initialize(const std::string& model_path, int inference_frequency_hz, bool use_cuda)
{
  (void)model_path;
  (void)inference_frequency_hz;
  (void)use_cuda;
  RCLCPP_ERROR(logger_, "ONNX Runtime not available. Cannot initialize inference.");
  return false;
}

void ONNXRuntimeInference::setTensorNames(const std::string& input_name,
                                           const std::string& actions_name,
                                           const std::string& lin_vel_name,
                                           const std::string& height_name)
{
  (void)input_name;
  (void)actions_name;
  (void)lin_vel_name;
  (void)height_name;
}

void ONNXRuntimeInference::start()
{
  RCLCPP_WARN(logger_, "ONNX Runtime not available. Cannot start inference.");
}

void ONNXRuntimeInference::stop()
{
  if (running_) {
    running_ = false;
    if (inference_thread_.joinable()) {
      inference_thread_.join();
    }
  }
}

void ONNXRuntimeInference::setInput(const std::vector<float>& input_data)
{
  (void)input_data;
  // No-op when ONNX Runtime is not available
}

bool ONNXRuntimeInference::getOutput(std::vector<float>& output_data)
{
  (void)output_data;
  return false;
}

void ONNXRuntimeInference::setMode(int mode, rcl_clock_type_t clock_type,
                                    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  (void)mode;
  (void)clock_type;
  (void)node;
}

void ONNXRuntimeInference::inferenceCallback()
{
  // No-op when ONNX Runtime is not available
}

void ONNXRuntimeInference::inferenceThread()
{
  // No-op when ONNX Runtime is not available
}

bool ONNXRuntimeInference::loadModelFromFile(const std::string& model_path)
{
  (void)model_path;
  return false;
}

bool ONNXRuntimeInference::createSession()
{
  return false;
}

void ONNXRuntimeInference::destroySession()
{
  // No-op
}

bool ONNXRuntimeInference::allocateBuffers()
{
  return false;
}

void ONNXRuntimeInference::freeBuffers()
{
  // No-op
}

}  // namespace robot_locomotion

#endif  // ONNXRUNTIME_AVAILABLE

