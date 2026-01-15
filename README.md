# wheelbipe_ros2_sim2sim

Wheelbipe 双轮机器人在 Webots 仿真环境下的 ROS 2 控制框架，支持 RL 训练与 sim2sim 流程。

## 环境要求

| 组件 | 版本/说明 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill (desktop) |
| 仿真器 | Webots ≥ R2023a |
| CUDA/TensorRT | CUDA 12.x + TensorRT 10.x (可选，用于 RL 推理) |
| ONNX Runtime | 1.20.x 或更高版本 (可选，用于 RL 推理，支持 CPU/GPU) |

## 快速开始

### 1. 安装依赖

```bash
# ROS 2 基础依赖
sudo apt install ros-humble-desktop ros-humble-webots-ros2-driver \
                 ros-humble-webots-ros2-control python3-colcon-common-extensions \
                 ros-humble-xacro

# TensorRT (可选，用于 TensorRT 后端)
sudo apt install nvidia-cuda-dev tensorrt-dev tensorrt

# ros2_control 扩展
# 参考: https://github.com/DDTRobot/TITA_ROS2_Control_Sim.git

# 安装TensorRT（参考版本为10.9）
# https://docs.nvidia.com/deeplearning/tensorrt/latest/installing-tensorrt/installing.html
# https://zhuanlan.zhihu.com/p/679763042

# ONNX Runtime (可选，用于 ONNX Runtime 后端)
# 方法 1: 从官方预编译包安装（推荐）
# 访问 https://github.com/microsoft/onnxruntime/releases
# 下载对应版本（CPU 或 GPU）：
#   wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.0/onnxruntime-linux-x64-gpu-1.20.0.tgz  # GPU版本
#   # 或
#   wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.0/onnxruntime-linux-x64-1.20.0.tgz    # CPU版本
#   tar -xzf onnxruntime-linux-x64-*.tgz
#   设置环境变量：
#   export ONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-*
#   export LD_LIBRARY_PATH=$ONNXRUNTIME_ROOT/lib:$LD_LIBRARY_PATH
```

### 2. 构建

```bash
# 克隆仓库
git clone <repository_url>
cd wheelbipe_ros2_sim2sim

# 配置环境
source /opt/ros/humble/setup.bash

# 构建
colcon build

# 使用
source install/setup.bash
```

### 3. 运行仿真

==运行ros2前需要先退出conda环境==

```bash
source install/setup.bash

# 启动 Webots 仿真
ros2 launch template_middleware template_bring_up.launch.py \
  prefix:=wheelbipe25_v3
```

**启动参数：**
- `prefix`: 机器人模型前缀（默认 `wheelbipe25_v3`）（通过前缀更换机器人）

### 4.键盘遥操作

键盘遥操作工具 (`keyboard_teleop`) 支持通过键盘控制机器人运动。详细使用说明请参考：[keyboard_teleop README](src/tools/keyboard_teleop/README.md)

### 5.一般使用流程

- rl仓库中通过play，自动完成.pt->.onnx模型文件导出

- **选择推理后端**：
  - **TensorRT 后端**：使用 tensorrt 的量化器量化模型
    ```bash
    <path_to_tensorrt>/bin/trtexec --onnx=<onnx_filename> --saveEngine=<engine_filename>
    ```
  - **ONNX Runtime 后端**：直接使用 .onnx 文件，无需转换

- 修改 `template_ros2_controller_parameters.yaml`，配置以下参数：
  - `rl_model_path`: 模型文件路径（.engine 或 .onnx，需要手动修改为本地的绝对路径）
  - `rl_inference_backend`: 推理后端（`"tensorrt"` 或 `"onnxruntime"`）
  - `rl_onnx_use_cuda`: 对于 ONNX Runtime，是否使用 GPU（`false` 表示使用 CPU，推荐）
  - `rl_input_name`: 输入张量名称（如 `"policy"`）
  - `rl_output_name`: 主输出张量名称（如 `"actions"`）

- 编译

```bash
source /opt/ros/humble/setup.bash && colcon build
source install/setup.bash
```

- 运行仿真

```bash
ros2 launch template_middleware template_bring_up.launch.py \
  prefix:=wheelbipe25_v3
```

- 启动键盘

```bash
ros2 run keyboard_teleop keyboard_teleop_node \
  --ros-args \
  --params-file $(ros2 pkg prefix keyboard_teleop)/share/keyboard_teleop/config/keyboard_teleop_params.yaml
```

## 仓库结构

```
src/
├── controllers/template_ros2_controller/    # RL 控制器（ros2_control 插件）
├── interfaces/webots_bridge/                 # Webots 桥接节点
├── middlewares/template_middleware/          # 启动文件与配置
├── resources/robot_descriptions/             # 机器人模型（URDF/Xacro、Webots 世界）
└── tools/keyboard_teleop/                    # 键盘遥操作工具
```

## 配置说明

### 控制器参数

主要配置文件：`src/controllers/template_ros2_controller/config/template_ros2_controller_parameters.yaml`

#### 推理后端配置

- `rl_model_path`: 模型文件路径（.engine 或 .onnx，需要手动修改为本地的绝对路径）
- `rl_inference_backend`: 推理后端类型
  - `"tensorrt"`: 使用 TensorRT 后端（需要 .engine 文件）
  - `"onnxruntime"`: 使用 ONNX Runtime 后端（需要 .onnx 文件）
- `rl_onnx_use_cuda`: 对于 ONNX Runtime，是否使用 GPU
  - `false`: 使用 CPU 推理（推荐，避免 CUBLAS 错误）
  - `true`: 使用 CUDA/GPU 推理（需要 CUDA 环境）
- `rl_input_name`: 输入张量名称（如 `"policy"`）
- `rl_output_name`: 主输出张量名称（如 `"actions"`）

#### 其他关键参数

- `rl_inference_frequency`: 推理频率 (Hz)
- `inference_mode` / `lowlevel_mode`: 调度模式（0=线程，1=ROS2 定时器，2=内联）
- `joint_stiffness` / `joint_damping`: 关节 PD 参数
- `joint_action_scale`: 动作缩放
- `joint_output_max` / `joint_output_min`: 输出限幅
- `joint_bias`: 关节偏置
- `default_dof_pos`: 默认关节位置

### 机器人模型

模型文件位于 `src/resources/robot_descriptions/<prefix>/`：
- `xacro/robot.xacro`: 机器人描述文件
- `worlds/*.wbt`: Webots 世界文件
- `meshes/*.STL`: 实体网格

## 常用命令

| 操作 | 命令 |
| --- | --- |
| 检查控制器状态 | `ros2 control list_controllers` |
| 查看关节状态 | `ros2 topic echo /<prefix>/joint_states` |
| 键盘遥操作 | `ros2 launch keyboard_teleop keyboard_teleop.launch.py` |
| 清理构建缓存 | `colcon build --packages-select <package> --cmake-clean-cache` |

## 故障排查

| 问题 | 解决方案 |
| --- | --- |
| `libfmt.a` 链接错误 | 检查环境变量，确保未引用 `/usr/local/lib/libfmt.a` |
| TensorRT 库未找到 | 设置 `export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH` |
| ONNX Runtime 库未找到 | 设置 `export ONNXRUNTIME_ROOT=/path/to/onnxruntime` 和 `export LD_LIBRARY_PATH=$ONNXRUNTIME_ROOT/lib:$LD_LIBRARY_PATH` |
| ONNX Runtime CUBLAS 错误 | 在 YAML 中设置 `rl_onnx_use_cuda: false` 使用 CPU 推理 |
| Webots 未启动 | 检查 `robot_descriptions/<prefix>/worlds/<prefix>.wbt` 是否存在 |
| 控制器加载失败 | 确认 `controllers.yaml` 与 `prefix` 匹配，插件名正确 |
| 输入/输出张量名称错误 | 检查模型文件，确认 `rl_input_name` 和 `rl_output_name` 与模型中的张量名称匹配 |

## 推理后端说明

### TensorRT 后端

- **模型格式**: `.engine` 文件（需要从 `.onnx` 转换）
- **优势**: 推理速度快，针对 NVIDIA GPU 优化
- **要求**: CUDA + TensorRT 环境
- **转换命令**: 
  ```bash
  <path_to_tensorrt>/bin/trtexec --onnx=<onnx_filename> --saveEngine=<engine_filename>
  ```

### ONNX Runtime 后端

- **模型格式**: `.onnx` 文件（直接使用，无需转换）
- **优势**: 通用性强，支持多种硬件（CPU/GPU），配置简单
- **要求**: ONNX Runtime 库（CPU 版本无需 CUDA）
- **推荐配置**: 
  - 使用 CPU 推理：`rl_onnx_use_cuda: false`（推荐，避免 CUBLAS 错误）
  - 使用 GPU 推理：`rl_onnx_use_cuda: true`（需要 CUDA 环境）

### 切换后端

只需在 `template_ros2_controller_parameters.yaml` 中修改：
- `rl_inference_backend`: `"tensorrt"` 或 `"onnxruntime"`
- `rl_model_path`: 对应的模型文件路径（.engine 或 .onnx）

## 参考

- [Webots ROS 2](https://github.com/cyberbotics/webots_ros2)
- [ros2_control](https://control.ros.org/)
- [TensorRT 文档](https://docs.nvidia.com/deeplearning/tensorrt/)
- [ONNX Runtime 文档](https://onnxruntime.ai/)
