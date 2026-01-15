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

#include "tensorrt_cuda/tensorrt_inference.hpp"
#include <fstream>
#include <iostream>
#include <cstring>
#include "rclcpp/create_timer.hpp"
#include "rcl/time.h"

#ifdef TENSORRT_AVAILABLE

namespace robot_locomotion
{

// TensorRT Logger
class TensorRTLogger : public nvinfer1::ILogger
{
public:
  TensorRTLogger(rclcpp::Logger logger) : logger_(logger) {}
  void log(Severity severity, const char* msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      RCLCPP_WARN(logger_, "TensorRT: %s", msg);
    } else {
      RCLCPP_DEBUG(logger_, "TensorRT: %s", msg);
    }
  }
private:
  rclcpp::Logger logger_;
};

TensorRTInference::TensorRTInference(rclcpp::Logger logger)
  : logger_(logger)
  , inference_frequency_hz_(100)
  , inference_period_ms_(10)
  , runtime_(nullptr)
  , engine_(nullptr)
  , context_(nullptr)
  , input_buffer_(nullptr)
  , output_actions_buffer_(nullptr)
  , output_lin_vel_buffer_(nullptr)
  , output_height_buffer_(nullptr)
  , input_size_(0)
  , output_actions_size_(0)
  , output_lin_vel_size_(0)
  , output_height_size_(0)
  , input_binding_index_(-1)
  , output_binding_index_(-1)
  , input_tensor_name_()
  , output_actions_name_()
  , output_lin_vel_name_()
  , output_height_name_()
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

TensorRTInference::~TensorRTInference()
{
  stop();
  destroyEngine();
  freeBuffers();
}

void TensorRTInference::setTensorNames(const std::string& input_name,
                                       const std::string& actions_name,
                                       const std::string& lin_vel_name,
                                       const std::string& height_name)
{
  // 仅保存“期望名称”，具体是否存在由 createExecutionContext 中进行检查和回退
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

bool TensorRTInference::initialize(const std::string& engine_model_path, int inference_frequency_hz)
{
  if (initialized_) {
    RCLCPP_WARN(logger_, "TensorRT inference already initialized");
    return true;
  }

  engine_model_path_ = engine_model_path;
  inference_frequency_hz_ = inference_frequency_hz;
  inference_period_ms_ = std::chrono::milliseconds(1000 / inference_frequency_hz_);
  period_nanoseconds_ = (1000000000LL / inference_frequency_hz_);  // 转换为纳秒

  // 检查文件是否存在
  std::ifstream file(engine_model_path, std::ios::binary);
  if (!file.good()) {
    RCLCPP_ERROR(logger_, "TensorRT engine file not found: %s", engine_model_path.c_str());
    return false;
  }
  file.close();

  // 加载 TensorRT engine
  if (!loadEngineFromFile(engine_model_path)) {
    RCLCPP_ERROR(logger_, "Failed to load TensorRT engine from file");
    return false;
  }

  // 创建执行上下文
  if (!createExecutionContext()) {
    RCLCPP_ERROR(logger_, "Failed to create TensorRT execution context");
    return false;
  }

  // 分配 CUDA 缓冲区
  if (!allocateBuffers()) {
    RCLCPP_ERROR(logger_, "Failed to allocate CUDA buffers");
    return false;
  }

  // 创建 CUDA 流
  if (cudaStreamCreate(&cuda_stream_) != cudaSuccess) {
    RCLCPP_ERROR(logger_, "Failed to create CUDA stream");
    return false;
  }

  initialized_ = true;
  RCLCPP_INFO(logger_, "TensorRT inference initialized successfully. Model: %s, Frequency: %d Hz",
    engine_model_path.c_str(), inference_frequency_hz_);
  
  return true;
}

bool TensorRTInference::loadEngineFromFile(const std::string& engine_model_path)
{
  static TensorRTLogger trt_logger(logger_);

  // 读取 engine 文件
  std::ifstream file(engine_model_path, std::ios::binary);
  if (!file.good()) {
    RCLCPP_ERROR(logger_, "Failed to open engine file: %s", engine_model_path.c_str());
    return false;
  }

  // 获取文件大小
  file.seekg(0, std::ios::end);
  size_t file_size = file.tellg();
  file.seekg(0, std::ios::beg);

  // 读取 engine 数据
  std::vector<char> engine_data(file_size);
  file.read(engine_data.data(), file_size);
  file.close();

  if (!file.good()) {
    RCLCPP_ERROR(logger_, "Failed to read engine file: %s", engine_model_path.c_str());
    return false;
  }

  // 创建 runtime
  runtime_ = nvinfer1::createInferRuntime(trt_logger);
  if (!runtime_) {
    RCLCPP_ERROR(logger_, "Failed to create TensorRT runtime");
    return false;
  }

  // 反序列化 engine (TensorRT 10 API: only 2 parameters)
  engine_ = runtime_->deserializeCudaEngine(engine_data.data(), file_size);
  if (!engine_) {
    RCLCPP_ERROR(logger_, "Failed to deserialize TensorRT engine");
    delete runtime_;
    runtime_ = nullptr;
    return false;
  }

  RCLCPP_INFO(logger_, "TensorRT engine loaded successfully from: %s", engine_model_path.c_str());
  return true;
}

bool TensorRTInference::createExecutionContext()
{
  if (!engine_) {
    RCLCPP_ERROR(logger_, "Engine not loaded, cannot create execution context");
    return false;
  }

  context_ = engine_->createExecutionContext();
  if (!context_) {
    RCLCPP_ERROR(logger_, "Failed to create execution context");
    return false;
  }

  // 获取输入输出张量名称 (TensorRT 10 API: use getNbIOTensors / getIOTensorName)
  // 新模型约定：输入为 policy，输出为 actions / lin_vel / height
  int32_t num_io_tensors = engine_->getNbIOTensors();

  std::string first_input_name;
  std::vector<std::string> output_tensor_names;

  for (int32_t i = 0; i < num_io_tensors; ++i) {
    const char* tensor_name = engine_->getIOTensorName(i);
    if (!tensor_name) {
      continue;
    }

    // 使用 TensorRT 的 I/O 模式接口判断输入/输出，避免仅靠名字猜测
    nvinfer1::TensorIOMode io_mode = engine_->getTensorIOMode(tensor_name);
    if (io_mode == nvinfer1::TensorIOMode::kINPUT) {
      if (first_input_name.empty()) {
        first_input_name = tensor_name;
      }
      // 如果还没有通过外部配置指定输入名，则优先尝试使用名为 "policy" 的张量
      if (input_tensor_name_.empty() && std::strcmp(tensor_name, "policy") == 0) {
        input_tensor_name_ = tensor_name;
      }
    } else if (io_mode == nvinfer1::TensorIOMode::kOUTPUT) {
      output_tensor_names.emplace_back(tensor_name);
      // 只有在尚未由外部配置指定时，才根据常用名称自动匹配
      if (output_actions_name_.empty() && std::strcmp(tensor_name, "actions") == 0) {
        output_actions_name_ = tensor_name;
      } else if (output_lin_vel_name_.empty() && std::strcmp(tensor_name, "lin_vel") == 0) {
        output_lin_vel_name_ = tensor_name;
      } else if (output_height_name_.empty() && std::strcmp(tensor_name, "height") == 0) {
        output_height_name_ = tensor_name;
      }
    }
  }

  // 兼容性：如果没有找到名为 policy 的输入，则退回第一个输入
  if (input_tensor_name_.empty()) {
    if (!first_input_name.empty()) {
      input_tensor_name_ = first_input_name;
      RCLCPP_WARN(logger_, "Input tensor 'policy' not found, using first input tensor: %s",
                  input_tensor_name_.c_str());
    } else {
      RCLCPP_ERROR(logger_, "No input tensors found in the engine");
      return false;
    }
  } else {
    RCLCPP_INFO(logger_, "Using input tensor: %s", input_tensor_name_.c_str());
  }

  // 兼容性：如果没有找到 actions/lin_vel/height，则根据输出顺序进行回退
  if (output_actions_name_.empty() && !output_tensor_names.empty()) {
    output_actions_name_ = output_tensor_names[0];
    RCLCPP_WARN(logger_, "Output tensor 'actions' not found, using first output tensor: %s",
                output_actions_name_.c_str());
  } else if (!output_actions_name_.empty()) {
    RCLCPP_INFO(logger_, "Using actions output tensor: %s", output_actions_name_.c_str());
  }

  // 其余两个输出是可选的：如果 engine 里有则使用，否则保持为空（旧模型只有一个输出）
  if (output_lin_vel_name_.empty()) {
    for (const auto& name : output_tensor_names) {
      if (name != output_actions_name_) {
        output_lin_vel_name_ = name;
        RCLCPP_WARN(logger_, "Output tensor 'lin_vel' not found, using additional output tensor: %s",
                    output_lin_vel_name_.c_str());
        break;
      }
    }
  }
  if (!output_lin_vel_name_.empty()) {
    RCLCPP_INFO(logger_, "Using lin_vel output tensor: %s", output_lin_vel_name_.c_str());
  }

  if (output_height_name_.empty()) {
    for (const auto& name : output_tensor_names) {
      if (name != output_actions_name_ && name != output_lin_vel_name_) {
        output_height_name_ = name;
        RCLCPP_WARN(logger_, "Output tensor 'height' not found, using additional output tensor: %s",
                    output_height_name_.c_str());
        break;
      }
    }
  }
  if (!output_height_name_.empty()) {
    RCLCPP_INFO(logger_, "Using height output tensor: %s", output_height_name_.c_str());
  }

  if (output_actions_name_.empty()) {
    RCLCPP_ERROR(logger_, "No usable output tensor found (expected at least 'actions')");
    return false;
  }

  return true;
}

bool TensorRTInference::allocateBuffers()
{
  if (!engine_ || input_tensor_name_.empty() || output_actions_name_.empty()) {
    RCLCPP_ERROR(logger_, "Engine not loaded or tensor names not set, cannot allocate buffers");
    return false;
  }

  // 获取输入尺寸 (TensorRT 10 API: use getTensorShape)
  auto input_dims = context_->getTensorShape(input_tensor_name_.c_str());

  input_size_ = 1;
  for (int i = 0; i < input_dims.nbDims; ++i) {
    input_size_ *= input_dims.d[i];
  }
  input_size_ *= sizeof(float);

  // actions 输出尺寸
  auto actions_dims = context_->getTensorShape(output_actions_name_.c_str());
  output_actions_size_ = 1;
  for (int i = 0; i < actions_dims.nbDims; ++i) {
    output_actions_size_ *= actions_dims.d[i];
  }
  output_actions_size_ *= sizeof(float);

  // lin_vel 输出尺寸（可选）
  if (!output_lin_vel_name_.empty()) {
    auto lin_vel_dims = context_->getTensorShape(output_lin_vel_name_.c_str());
    output_lin_vel_size_ = 1;
    for (int i = 0; i < lin_vel_dims.nbDims; ++i) {
      output_lin_vel_size_ *= lin_vel_dims.d[i];
    }
    output_lin_vel_size_ *= sizeof(float);
  } else {
    output_lin_vel_size_ = 0;
  }

  // height 输出尺寸（可选）
  if (!output_height_name_.empty()) {
    auto height_dims = context_->getTensorShape(output_height_name_.c_str());
    output_height_size_ = 1;
    for (int i = 0; i < height_dims.nbDims; ++i) {
      output_height_size_ *= height_dims.d[i];
    }
    output_height_size_ *= sizeof(float);
  } else {
    output_height_size_ = 0;
  }

  // 分配 CUDA 内存
  if (cudaMalloc(&input_buffer_, input_size_) != cudaSuccess) {
    RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for input");
    return false;
  }

  if (cudaMalloc(&output_actions_buffer_, output_actions_size_) != cudaSuccess) {
    RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for output");
    cudaFree(input_buffer_);
    return false;
  }

  if (output_lin_vel_size_ > 0) {
    if (cudaMalloc(&output_lin_vel_buffer_, output_lin_vel_size_) != cudaSuccess) {
      RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for lin_vel output");
      cudaFree(input_buffer_);
      cudaFree(output_actions_buffer_);
      return false;
    }
  }

  if (output_height_size_ > 0) {
    if (cudaMalloc(&output_height_buffer_, output_height_size_) != cudaSuccess) {
      RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for height output");
      cudaFree(input_buffer_);
      cudaFree(output_actions_buffer_);
      if (output_lin_vel_buffer_) {
        cudaFree(output_lin_vel_buffer_);
      }
      return false;
    }
  }

  // 初始化数据缓冲区
  input_data_.resize(input_size_ / sizeof(float));
  // 对外接口目前只需要 actions，因此仅为 actions 分配主机缓冲
  output_data_.resize(output_actions_size_ / sizeof(float));

  RCLCPP_INFO(logger_, "CUDA buffers allocated. Input size: %zu, actions: %zu, lin_vel: %zu, height: %zu (floats)",
    input_size_ / sizeof(float),
    output_actions_size_ / sizeof(float),
    output_lin_vel_size_ / sizeof(float),
    output_height_size_ / sizeof(float));

  return true;
}

void TensorRTInference::freeBuffers()
{
  if (input_buffer_) {
    cudaFree(input_buffer_);
    input_buffer_ = nullptr;
  }
  if (output_actions_buffer_) {
    cudaFree(output_actions_buffer_);
    output_actions_buffer_ = nullptr;
  }
  if (output_lin_vel_buffer_) {
    cudaFree(output_lin_vel_buffer_);
    output_lin_vel_buffer_ = nullptr;
  }
  if (output_height_buffer_) {
    cudaFree(output_height_buffer_);
    output_height_buffer_ = nullptr;
  }
  if (cuda_stream_) {
    cudaStreamDestroy(cuda_stream_);
    cuda_stream_ = nullptr;
  }
}

void TensorRTInference::destroyEngine()
{
  // TensorRT 10: 使用 delete 而不是 destroy()
  if (context_) {
    delete context_;
    context_ = nullptr;
  }
  if (engine_) {
    delete engine_;
    engine_ = nullptr;
  }
  if (runtime_) {
    delete runtime_;
    runtime_ = nullptr;
  }
}

void TensorRTInference::start()
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
    RCLCPP_INFO(logger_, "TensorRT inference inline mode (no thread/timer)");
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
        std::bind(&TensorRTInference::inferenceCallback, this),
        nullptr);
      RCLCPP_INFO(logger_, "TensorRT inference started with ROS2 timer (mode=1, clock=%d, period=%ld ns)",
                  static_cast<int>(timer_clock_type_), period_nanoseconds_);
      return;
    } else {
      RCLCPP_WARN(logger_, "Period not set, fallback to thread mode");
      mode_ = 0;
    }
  }

  // 默认：线程+系统时间
  inference_thread_ = std::thread(&TensorRTInference::inferenceThread, this);
  RCLCPP_INFO(logger_, "TensorRT inference thread started (mode=0 system time)");
}

void TensorRTInference::stop()
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

  RCLCPP_INFO(logger_, "TensorRT inference stopped");
}

void TensorRTInference::setInput(const std::vector<float>& input_data)
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

bool TensorRTInference::getOutput(std::vector<float>& output_data)
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

void TensorRTInference::setMode(int mode, rcl_clock_type_t clock_type,
                                rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  mode_ = mode;
  timer_clock_type_ = clock_type;
  node_ = node;
}

void TensorRTInference::inferenceCallback()
{
  // ROS2 定时器回调函数
  auto current_time = std::chrono::steady_clock::now();
  
  // 计算并打印两次执行之间的时间间隔
  // if (last_execution_time_.time_since_epoch().count() > 0)
  // {
  //   auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(
  //       current_time - last_execution_time_);
  //   RCLCPP_INFO(logger_, "inferenceThread execution interval: %ld us (%.3f ms)", 
  //               static_cast<long>(time_diff.count()), time_diff.count() / 1000.0);
  // }
  last_execution_time_ = current_time;

  // 执行推理
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (new_input_available_ && input_ready_) {
    // 复制输入数据到 CUDA 缓冲区
    cudaMemcpyAsync(input_buffer_, input_data_.data(), input_size_,
                   cudaMemcpyHostToDevice, cuda_stream_);

    // TensorRT 10 API: 使用 setTensorAddress 设置张量地址
    if (!input_tensor_name_.empty() && !output_actions_name_.empty()) {
      context_->setTensorAddress(input_tensor_name_.c_str(), input_buffer_);
      context_->setTensorAddress(output_actions_name_.c_str(), output_actions_buffer_);
      // 其余输出张量（如 lin_vel / height）也需要绑定地址，否则 TensorRT 会报错
      if (!output_lin_vel_name_.empty() && output_lin_vel_buffer_) {
        context_->setTensorAddress(output_lin_vel_name_.c_str(), output_lin_vel_buffer_);
      }
      if (!output_height_name_.empty() && output_height_buffer_) {
        context_->setTensorAddress(output_height_name_.c_str(), output_height_buffer_);
      }
    } else {
      RCLCPP_ERROR(logger_, "Tensor names not set");
      new_input_available_ = false;
      return;
    }

    // 执行推理 (TensorRT 10 API: use enqueueV3)
    bool success = context_->enqueueV3(cuda_stream_);
    
    if (!success) {
      RCLCPP_ERROR(logger_, "TensorRT inference failed");
      new_input_available_ = false;
      return;
    }

    // 同步 CUDA 流
    cudaStreamSynchronize(cuda_stream_);

    // 复制 actions 输出数据回主机
    cudaMemcpyAsync(output_data_.data(), output_actions_buffer_, output_actions_size_,
                   cudaMemcpyDeviceToHost, cuda_stream_);
    cudaStreamSynchronize(cuda_stream_);

    output_ready_ = true;
    new_input_available_ = false;
    inference_done_cv_.notify_one();
  }
}

void TensorRTInference::inferenceThread()
{
  RCLCPP_INFO(logger_, "TensorRT inference thread started");

  auto next_inference_time = std::chrono::steady_clock::now();
  last_execution_time_ = std::chrono::steady_clock::time_point{};

  while (running_) {
    auto current_time = std::chrono::steady_clock::now();
    
    // 计算并打印两次执行之间的时间间隔
    // if (last_execution_time_.time_since_epoch().count() > 0)
    // {
    //   auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(
    //       current_time - last_execution_time_);
    //   RCLCPP_INFO(logger_, "inferenceThread execution interval: %ld us (%.3f ms)", 
    //               static_cast<long>(time_diff.count()), time_diff.count() / 1000.0);
    // }
    last_execution_time_ = current_time;
    
    // 等待输入数据或停止信号
    std::unique_lock<std::mutex> lock(data_mutex_);
    data_ready_cv_.wait(lock, [this] { return new_input_available_ || !running_; });

    if (!running_) {
      break;
    }

    if (new_input_available_ && input_ready_) {
      // 复制输入数据到 CUDA 缓冲区
      cudaMemcpyAsync(input_buffer_, input_data_.data(), input_size_,
                     cudaMemcpyHostToDevice, cuda_stream_);

      // TensorRT 10 API: 使用 setTensorAddress 设置张量地址
      if (!input_tensor_name_.empty() && !output_actions_name_.empty()) {
        context_->setTensorAddress(input_tensor_name_.c_str(), input_buffer_);
        context_->setTensorAddress(output_actions_name_.c_str(), output_actions_buffer_);
        if (!output_lin_vel_name_.empty() && output_lin_vel_buffer_) {
          context_->setTensorAddress(output_lin_vel_name_.c_str(), output_lin_vel_buffer_);
        }
        if (!output_height_name_.empty() && output_height_buffer_) {
          context_->setTensorAddress(output_height_name_.c_str(), output_height_buffer_);
        }
      } else {
        RCLCPP_ERROR(logger_, "Tensor names not set");
        new_input_available_ = false;
        continue;
      }

      // 执行推理 (TensorRT 10 API: use enqueueV3)
      bool success = context_->enqueueV3(cuda_stream_);
      
      if (!success) {
        RCLCPP_ERROR(logger_, "TensorRT inference failed");
        new_input_available_ = false;
        continue;
      }

      // 同步 CUDA 流
      cudaStreamSynchronize(cuda_stream_);

      // 复制 actions 输出数据回主机
      cudaMemcpyAsync(output_data_.data(), output_actions_buffer_, output_actions_size_,
                     cudaMemcpyDeviceToHost, cuda_stream_);
      cudaStreamSynchronize(cuda_stream_);

      output_ready_ = true;
      new_input_available_ = false;
      inference_done_cv_.notify_one();
    }

    lock.unlock();

    // 使用系统时间进行频率控制（仅在不使用 ROS2 定时器时）
    next_inference_time += inference_period_ms_;
    std::this_thread::sleep_until(next_inference_time);
  }

  RCLCPP_INFO(logger_, "TensorRT inference thread stopped");
}

}  // namespace robot_locomotion

#else  // TENSORRT_AVAILABLE not defined

// Stub implementation when TensorRT is not available
namespace robot_locomotion
{

TensorRTInference::TensorRTInference(rclcpp::Logger logger)
  : logger_(logger)
  , inference_frequency_hz_(100)
  , inference_period_ms_(10)
  , runtime_(nullptr)
  , engine_(nullptr)
  , context_(nullptr)
  , input_buffer_(nullptr)
  , output_actions_buffer_(nullptr)
  , output_lin_vel_buffer_(nullptr)
  , output_height_buffer_(nullptr)
  , input_size_(0)
  , output_actions_size_(0)
  , output_lin_vel_size_(0)
  , output_height_size_(0)
  , input_binding_index_(-1)
  , output_binding_index_(-1)
  , input_tensor_name_()
  , output_actions_name_()
  , output_lin_vel_name_()
  , output_height_name_()
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
  RCLCPP_WARN(logger_, "TensorRT not available. RL inference will not work.");
}

TensorRTInference::~TensorRTInference()
{
  stop();
}

bool TensorRTInference::initialize(const std::string& engine_model_path, int inference_frequency_hz)
{
  (void)engine_model_path;
  (void)inference_frequency_hz;
  RCLCPP_ERROR(logger_, "TensorRT not available. Cannot initialize inference.");
  return false;
}

void TensorRTInference::start()
{
  RCLCPP_WARN(logger_, "TensorRT not available. Cannot start inference.");
}

void TensorRTInference::stop()
{
  if (running_) {
    running_ = false;
    if (inference_thread_.joinable()) {
      inference_thread_.join();
    }
  }
}

void TensorRTInference::setInput(const std::vector<float>& input_data)
{
  (void)input_data;
  // No-op when TensorRT is not available
}

bool TensorRTInference::getOutput(std::vector<float>& output_data)
{
  (void)output_data;
  return false;
}

void TensorRTInference::inferenceThread()
{
  // No-op when TensorRT is not available
}

bool TensorRTInference::loadEngineFromFile(const std::string& engine_model_path)
{
  (void)engine_model_path;
  return false;
}

bool TensorRTInference::createExecutionContext()
{
  return false;
}

void TensorRTInference::destroyEngine()
{
  // No-op
}

bool TensorRTInference::allocateBuffers()
{
  return false;
}

void TensorRTInference::freeBuffers()
{
  // No-op
}

}  // namespace robot_locomotion

#endif  // TENSORRT_AVAILABLE

