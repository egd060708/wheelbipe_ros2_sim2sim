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

#include "tensorrt_cuda/tensorrt_inference.hpp"
#include <fstream>
#include <iostream>
#include <cstring>

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
  , output_buffer_(nullptr)
  , input_size_(0)
  , output_size_(0)
  , input_binding_index_(-1)
  , output_binding_index_(-1)
  , input_tensor_name_()
  , output_tensor_name_()
  , running_(false)
  , initialized_(false)
  , input_ready_(false)
  , output_ready_(false)
  , new_input_available_(false)
  , cuda_stream_(nullptr)
  , last_execution_time_{}
{
}

TensorRTInference::~TensorRTInference()
{
  stop();
  destroyEngine();
  freeBuffers();
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

  // 获取输入输出张量名称 (TensorRT 10 API: use getNbIOTensors and getIOTensorName)
  // 尝试使用常见的输入输出名称
  const char* input_names[] = {"input", "obs", "observation", "state"};
  const char* output_names[] = {"output", "action", "actions", "torque"};
  
  std::string input_tensor_name;
  std::string output_tensor_name;
  
  // 遍历所有 IO 张量，查找输入和输出
  int32_t num_io_tensors = engine_->getNbIOTensors();
  for (int32_t i = 0; i < num_io_tensors; ++i) {
    const char* tensor_name = engine_->getIOTensorName(i);
    if (!tensor_name) continue;
    
    // 检查是否是输入
    for (const char* name : input_names) {
      if (std::strcmp(tensor_name, name) == 0) {
        input_tensor_name = tensor_name;
        RCLCPP_INFO(logger_, "Found input tensor: %s", tensor_name);
        break;
      }
    }
    
    // 检查是否是输出
    for (const char* name : output_names) {
      if (std::strcmp(tensor_name, name) == 0) {
        output_tensor_name = tensor_name;
        RCLCPP_INFO(logger_, "Found output tensor: %s", tensor_name);
        break;
      }
    }
  }

  // 如果没有找到，使用第一个作为输入，第二个作为输出
  if (input_tensor_name.empty() && num_io_tensors > 0) {
    input_tensor_name = engine_->getIOTensorName(0);
    RCLCPP_WARN(logger_, "Using first tensor as input: %s", input_tensor_name.c_str());
  }
  if (output_tensor_name.empty() && num_io_tensors > 1) {
    output_tensor_name = engine_->getIOTensorName(1);
    RCLCPP_WARN(logger_, "Using second tensor as output: %s", output_tensor_name.c_str());
  }

  // 存储张量名称（用于后续设置地址）
  input_tensor_name_ = input_tensor_name;
  output_tensor_name_ = output_tensor_name;

  return true;
}

bool TensorRTInference::allocateBuffers()
{
  if (!engine_ || input_tensor_name_.empty() || output_tensor_name_.empty()) {
    RCLCPP_ERROR(logger_, "Engine not loaded or tensor names not set, cannot allocate buffers");
    return false;
  }

  // 获取输入输出尺寸 (TensorRT 10 API: use getTensorShape)
  auto input_dims = context_->getTensorShape(input_tensor_name_.c_str());
  auto output_dims = context_->getTensorShape(output_tensor_name_.c_str());

  input_size_ = 1;
  for (int i = 0; i < input_dims.nbDims; ++i) {
    input_size_ *= input_dims.d[i];
  }
  input_size_ *= sizeof(float);

  output_size_ = 1;
  for (int i = 0; i < output_dims.nbDims; ++i) {
    output_size_ *= output_dims.d[i];
  }
  output_size_ *= sizeof(float);

  // 分配 CUDA 内存
  if (cudaMalloc(&input_buffer_, input_size_) != cudaSuccess) {
    RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for input");
    return false;
  }

  if (cudaMalloc(&output_buffer_, output_size_) != cudaSuccess) {
    RCLCPP_ERROR(logger_, "Failed to allocate CUDA memory for output");
    cudaFree(input_buffer_);
    return false;
  }

  // 初始化数据缓冲区
  input_data_.resize(input_size_ / sizeof(float));
  output_data_.resize(output_size_ / sizeof(float));

  RCLCPP_INFO(logger_, "CUDA buffers allocated. Input size: %zu, Output size: %zu",
    input_size_ / sizeof(float), output_size_ / sizeof(float));

  return true;
}

void TensorRTInference::freeBuffers()
{
  if (input_buffer_) {
    cudaFree(input_buffer_);
    input_buffer_ = nullptr;
  }
  if (output_buffer_) {
    cudaFree(output_buffer_);
    output_buffer_ = nullptr;
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
  inference_thread_ = std::thread(&TensorRTInference::inferenceThread, this);
  RCLCPP_INFO(logger_, "TensorRT inference thread started");
}

void TensorRTInference::stop()
{
  if (!running_) {
    return;
  }

  running_ = false;
  data_ready_cv_.notify_all();

  if (inference_thread_.joinable()) {
    inference_thread_.join();
  }

  RCLCPP_INFO(logger_, "TensorRT inference thread stopped");
}

void TensorRTInference::setInput(const std::vector<float>& input_data)
{
  if (!initialized_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (input_data.size() != input_data_.size()) {
    static rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(logger_, clock, 1000,
      "Input size mismatch: expected %zu, got %zu", input_data_.size(), input_data.size());
    return;
  }

  input_data_ = input_data;
  new_input_available_ = true;
  input_ready_ = true;
  data_ready_cv_.notify_one();
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

void TensorRTInference::inferenceThread()
{
  RCLCPP_INFO(logger_, "TensorRT inference thread started");

  auto next_inference_time = std::chrono::steady_clock::now();
  last_execution_time_ = std::chrono::steady_clock::time_point{};

  while (running_) {
    // auto current_time = std::chrono::steady_clock::now();
    
    // // 计算并打印两次执行之间的时间间隔
    // if (last_execution_time_.time_since_epoch().count() > 0)
    // {
    //   auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(
    //       current_time - last_execution_time_);
    //   RCLCPP_INFO(logger_, "inferenceThread execution interval: %lld us (%.3f ms)", 
    //               time_diff.count(), time_diff.count() / 1000.0);
    // }
    // last_execution_time_ = current_time;
    
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
      if (!input_tensor_name_.empty() && !output_tensor_name_.empty()) {
        context_->setTensorAddress(input_tensor_name_.c_str(), input_buffer_);
        context_->setTensorAddress(output_tensor_name_.c_str(), output_buffer_);
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

      // 复制输出数据回主机
      cudaMemcpyAsync(output_data_.data(), output_buffer_, output_size_,
                     cudaMemcpyDeviceToHost, cuda_stream_);
      cudaStreamSynchronize(cuda_stream_);

      output_ready_ = true;
      new_input_available_ = false;
      inference_done_cv_.notify_one();
    }

    lock.unlock();

    // 控制推理频率
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
  , output_buffer_(nullptr)
  , input_size_(0)
  , output_size_(0)
  , input_binding_index_(-1)
  , output_binding_index_(-1)
  , input_tensor_name_()
  , output_tensor_name_()
  , running_(false)
  , initialized_(false)
  , input_ready_(false)
  , output_ready_(false)
  , new_input_available_(false)
  , cuda_stream_(nullptr)
  , last_execution_time_{}
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

