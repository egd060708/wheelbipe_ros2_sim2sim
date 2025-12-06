# wheelbipe_ros2_sim2sim

Wheelbipe 双轮机器人在 Webots 仿真环境下的 ROS 2 控制框架，支持 RL 训练与 sim2sim 流程。

## 环境要求

| 组件 | 版本/说明 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill (desktop) |
| 仿真器 | Webots ≥ R2023a |
| CUDA/TensorRT | CUDA 12.x + TensorRT 10.x (可选，用于 RL 推理) |

## 快速开始

### 1. 安装依赖

```bash
# ROS 2 基础依赖
sudo apt install ros-humble-desktop ros-humble-webots-ros2-driver \
                 ros-humble-webots-ros2-control python3-colcon-common-extensions \
                 ros-humble-xacro

# TensorRT (可选)
sudo apt install nvidia-cuda-dev tensorrt-dev tensorrt

# ros2_control 扩展
# 参考: https://github.com/DDTRobot/TITA_ROS2_Control_Sim.git

# 安装TensorRT（参考版本为10.9）
# https://docs.nvidia.com/deeplearning/tensorrt/latest/installing-tensorrt/installing.html
# https://zhuanlan.zhihu.com/p/679763042
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

- 使用tensorrt的量化器量化模型

```bash
<path_to_tensorrt>/bin/trtexec --onnx=<onnx_filename> --saveEngine=<engine_filename>
```

- 修改template_ros2_controller_parameters.yaml，注意配置`rl_model_path`为本地绝对路径
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

关键参数：
- `rl_model_path`: TensorRT 引擎文件路径==（需要手动修改为本地的绝对路径）==
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
| Webots 未启动 | 检查 `robot_descriptions/<prefix>/worlds/<prefix>.wbt` 是否存在 |
| 控制器加载失败 | 确认 `controllers.yaml` 与 `prefix` 匹配，插件名正确 |

## 参考

- [Webots ROS 2](https://github.com/cyberbotics/webots_ros2)
- [ros2_control](https://control.ros.org/)
- [TensorRT 文档](https://docs.nvidia.com/deeplearning/tensorrt/)
