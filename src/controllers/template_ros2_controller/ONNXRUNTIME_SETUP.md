# ONNX Runtime 安装与使用指南

## 推荐的 ONNX Runtime 版本

- **推荐版本**: ONNX Runtime 1.20.x 或更高版本
- **兼容性**: 
  - CUDA 12.x + cuDNN 9.x: ONNX Runtime ≥ 1.19.x（推荐 1.20.x+）
  - CUDA 11.x + cuDNN 8.x: ONNX Runtime ≥ 1.18.x
  - CPU-only: 任意最新版本

## 安装 ONNX Runtime（CUDA 版本）

### 方法 1: 从官方预编译包安装（推荐）

1. 访问 [ONNX Runtime 发布页面](https://github.com/microsoft/onnxruntime/releases)
2. 下载对应 CUDA 版本的预编译包，例如：
   ```bash
   # 对于 CUDA 12.x
   wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.0/onnxruntime-linux-x64-gpu-1.20.0.tgz
   
   # 解压
   tar -xzf onnxruntime-linux-x64-gpu-1.20.0.tgz
   ```

3. 将库文件复制到系统路径或设置环境变量：
   ```bash
   # 复制到系统路径（需要 sudo）
   sudo cp -r onnxruntime-linux-x64-gpu-1.20.0/include/* /usr/local/include/onnxruntime/
   sudo cp -r onnxruntime-linux-x64-gpu-1.20.0/lib/* /usr/local/lib/
   
   # 或者设置环境变量（推荐）
   export ONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-gpu-1.20.0
   export LD_LIBRARY_PATH=$ONNXRUNTIME_ROOT/lib:$LD_LIBRARY_PATH
   ```

### 方法 2: 从源码编译（高级用户）

```bash
git clone --recursive https://github.com/microsoft/onnxruntime.git
cd onnxruntime
./build.sh --config Release --build_shared_lib --parallel --use_cuda --cuda_home /usr/local/cuda --cudnn_home /usr/local/cuda
```

## 配置 CMake 查找路径

如果 ONNX Runtime 安装在非标准路径，可以通过以下方式设置：

```bash
# 设置环境变量
export ONNXRUNTIME_ROOT=/path/to/onnxruntime

# 或者在 CMakeLists.txt 中直接指定路径
# 修改 CMakeLists.txt 中的 find_path 和 find_library 路径
```

## 使用 ONNX Runtime 推理

### 1. 修改 YAML 配置文件

在 `template_ros2_controller_parameters.yaml` 中设置：

```yaml
template_ros2_controller:
  rl_model_path:
    {
      type: string,
      default_value: "/path/to/your/model.onnx",  # 使用 .onnx 文件
      description: "Path to model file for reinforcement learning inference",
    }
  rl_inference_backend:
    {
      type: string,
      default_value: "onnxruntime",  # 设置为 "onnxruntime" 或 "onnx"
      description: "Inference backend: 'tensorrt' or 'onnxruntime'",
    }
  rl_input_name:
    {
      type: string,
      default_value: "policy",
      description: "Name of the input tensor",
    }
  rl_output_name:
    {
      type: string,
      default_value: "actions",
      description: "Name of the main output tensor (e.g., actions)",
    }
```

### 2. 编译项目

```bash
cd /home/lu/Git_Project/github/wheelbipe_ros2_sim2sim
colcon build --packages-select template_ros2_controller
source install/setup.bash
```

### 3. 运行

确保 ONNX Runtime 库在 `LD_LIBRARY_PATH` 中，然后正常启动你的控制器。

## 后端切换

- **TensorRT**: 设置 `rl_inference_backend: "tensorrt"`，使用 `.engine` 文件
- **ONNX Runtime**: 设置 `rl_inference_backend: "onnxruntime"`，使用 `.onnx` 文件

## 注意事项

1. **CUDA 版本兼容性**: 确保 ONNX Runtime 的 CUDA 版本与系统 CUDA 版本匹配
2. **库路径**: 如果运行时找不到库，确保 `LD_LIBRARY_PATH` 包含 ONNX Runtime 的 lib 目录
3. **模型格式**: ONNX Runtime 使用 `.onnx` 格式，TensorRT 使用 `.engine` 格式
4. **性能**: TensorRT 通常比 ONNX Runtime 更快，但 ONNX Runtime 更通用，支持更多模型格式

## 故障排除

### 问题: CMake 找不到 ONNX Runtime

**解决方案**: 
- 检查 `ONNXRUNTIME_ROOT` 环境变量
- 确认头文件在 `$ONNXRUNTIME_ROOT/include/onnxruntime/` 目录下
- 确认库文件在 `$ONNXRUNTIME_ROOT/lib/` 目录下

### 问题: 运行时找不到 libonnxruntime.so

**解决方案**:
```bash
export LD_LIBRARY_PATH=$ONNXRUNTIME_ROOT/lib:$LD_LIBRARY_PATH
# 或者添加到 ~/.bashrc
echo 'export LD_LIBRARY_PATH=$ONNXRUNTIME_ROOT/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
```

### 问题: CUDA Execution Provider 初始化失败

**解决方案**:
- 检查 CUDA 和 cuDNN 版本是否兼容
- 确认 GPU 可用: `nvidia-smi`
- ONNX Runtime 会自动回退到 CPU，但性能会下降

