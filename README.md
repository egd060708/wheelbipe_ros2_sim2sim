# wheelbipe_ros2_sim2sim

Wheelbipe 双轮机器人在多仿真环境下用于 RL 训练与控制的 ROS 2 工程。仓库提供了 Webots 世界、机器人描述、ROS 控制器以及桥接节点，可在纯仿真环境中复现 sim2sim 流程。

---

## 环境与依赖

| 组件 | 版本 & 说明 |
| --- | --- |
| 操作系统 | Ubuntu 22.04（推荐） |
| ROS 2 | Humble Hawksbill（desktop 版，包含 `ros2-control`、`webots_ros2_driver` 等） |
| 构建工具 | `colcon`, `ament_cmake_auto` |
| 仿真器 | [Webots ≥ R2023a](https://cyberbotics.com/) |
| CUDA / TensorRT (可选) | 若使用 RL 推理（`template_ros2_controller` 内启用 TensorRT）需安装 CUDA 12.x 与 TensorRT 10.x，并设置 `LD_LIBRARY_PATH` 指向 `libnvinfer.so` 等库 |
| 其他 | `xacro`, `python3-colcon-common-extensions`, `pip install webots-ros2`（若未通过 apt 安装） |

> **提示**：如果系统或 Conda 环境里同时安装了多个 `libfmt` 版本，请确保编译时优先使用系统的共享库（仓库中的 `CMakeLists.txt` 已屏蔽 `/usr/local/lib/libfmt.a`）。

---

## 仓库结构

```
src/
├── controllers/
│   └── template_ros2_controller/   # RL 控制器（ros2_control 插件，含 TensorRT 推理）
├── interfaces/
│   └── webots_bridge/              # 启动 Webots 并桥接 ros2_control 的节点
├── middlewares/
│   └── template_middleware/        # 中间件/工具脚本（占位）
└── resources/
    └── robot_descriptions/         # Wheelbipe25 v3 模型（URDF/Xacro、Webots 世界、STL）
```

关键文件与配置：

- `config/template_ros2_controller_parameters.yaml`：控制器参数（关节 PD、推理频率、时间同步方式等）。
- `interfaces/webots_bridge/config/controllers.yaml`：`ros2_control` 控制器组合。
- `resources/robot_descriptions/wheelbipe25_v3/worlds/*.wbt`：Webots 世界文件。
- `interfaces/webots_bridge/launch/webots_bridge.launch.py`：统一启动 Webots + ros2_control + Robot State Publisher。

---

## 构建步骤

安装ros2_control   
https://github.com/DDTRobot/TITA_ROS2_Control_Sim.git

安装TensorRT

https://docs.nvidia.com/deeplearning/tensorrt/latest/installing-tensorrt/installing.html

https://zhuanlan.zhihu.com/p/679763042

```bash
sudo apt install nvidia-cuda-dev
sudo apt install tensorrt-dev
sudo apt install tensorrt
```

```bash
# 1. 安装依赖（示例）
sudo apt update
sudo apt install ros-humble-desktop ros-humble-webots-ros2-driver \
                 ros-humble-webots-ros2-control python3-colcon-common-extensions \
                 ros-humble-xacro

# 2. 克隆仓库
git clone https://github.com/<your_org>/wheelbipe_ros2_sim2sim.git
cd wheelbipe_ros2_sim2sim

# 3. 配置环境变量（每次新终端）
source /opt/ros/humble/setup.bash

# 4. 构建
colcon build --packages-up-to robot_descriptions webots_bridge template_middleware template_ros2_controller

# 5. 使用结果
source install/setup.bash
```

若启用 TensorRT，请确保：

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH   # 包含 libnvinfer.so
```

---

## 运行 Webots 仿真

1. 启动 Webots + ros2_control：

   ```bash
   source install/setup.bash
   ros2 launch webots_bridge webots_bridge.launch.py \
     prefix:=wheelbipe25_v3 \
     urdf:=robot.xacro \
     ctrl_mode:=wbc \
     yaml_path:=webots_bridge
   ```

   参数说明：

   - `prefix`：选择 `robot_descriptions/<prefix>` 下的模型与 world（默认 `wheelbipe25_v3`）。
   - `ctrl_mode`：`wbc`（默认）、`sdk`、`mcu`，传递给 `xacro` 以切换控制接口。
   - `yaml_path`：控制器配置所在包名（默认 `webots_bridge`，即 `interfaces/webots_bridge/config/<prefix>.yaml`）。

2. 启动后将：
   - 打开对应的 `.wbt` 世界
   - 自动加载 `robot_state_publisher`
   - 通过 `webots_ros2_driver` + `webots_ros2_control` 实例化 `template_ros2_controller`

---

## 控制器与参数

### `template_ros2_controller`

- 由 `ros2_control` 加载的自定义控制器，路径：`src/controllers/template_ros2_controller`.
- 内置 TensorRT 推理接口（`src/tensorrt_cuda/tensorrt_inference.cpp`）。若 `TENSORRT_AVAILABLE` 未定义，则自动降级为占位实现。
- 关键参数位于 `config/template_ros2_controller_parameters.yaml`：
  - `rl_model_path`：TensorRT 引擎路径。
  - `rl_inference_frequency`：推理频率（Hz）。
  - `use_period_for_inference_timing` / `use_period_for_lowlevel_timing`：选择基于 ROS 2 `period` 还是系统时间调度。
  - 关节/车轮 PD 参数、弹簧力等。

如需更新参数，可修改 yaml 后重新 `colcon build` 或在运行时通过 `ros2 param` 动态修改（控制器会检测 `param_listener_->is_old()`）。

### `webots_bridge`

- 主要负责启动 Webots、生成 robot_description、实例化 `webots_ros2_driver` 控制器以及 `robot_state_publisher`。
- 配置文件：`interfaces/webots_bridge/config/controllers.yaml`（定义 ros2_control 控制器组合、接口类型等）。

### Robot descriptions

- 位于 `resources/robot_descriptions/wheelbipe25_v3`：
  - `xacro/robot.xacro`：机器人主体，参数化控制模式、传感器等。
  - `worlds/*.wbt`：多种地形/场景。
  - `meshes/*.STL`：实体网格。

可复制该模板目录，以不同 `prefix` 创建新机器人/场景。

---

## 常见命令速查

| 操作 | 命令 |
| --- | --- |
| 清理单个包构建 | `colcon build --packages-select template_ros2_controller --cmake-clean-cache` |
| 仅启动 Webots （默认参数） | `ros2 launch webots_bridge webots_bridge.launch.py` |
| 指定世界 & 控制模式 | `ros2 launch webots_bridge webots_bridge.launch.py prefix:=wheelbipe25_v3 ctrl_mode:=sdk` |
| 检查控制器状态 | `ros2 control list_controllers` |
| 查看关节状态/命令 | `ros2 topic echo /wheelbipe25_v3/joint_states` |

---

## 故障排查

| 问题 | 解决方案 |
| --- | --- |
| 链接时报 `libfmt.a` / `-fPIC` 相关错误 | 确保未引用 `/usr/local/lib/libfmt.a`（仓库 CMake 已默认排除）。若仍报错，检查环境变量/Conda 是否注入额外路径。 |
| 找不到 TensorRT 或 CUDA 库 | 确认 `LD_LIBRARY_PATH` 包含 `/usr/lib/x86_64-linux-gnu`（或系统 TensorRT 路径），或在 `config/template_ros2_controller_parameters.yaml` 中暂时禁用 RL 推理。 |
| `webots_ros2_driver` 无法连接控制器 | 检查 `controllers.yaml` 与 `prefix` 是否匹配；确认 `ros2_control` 插件名（`template_ros2_controller/TemplateRos2Controller`）正确。 |
| Webots 未启动或自动退出 | 使用 `--ros-args --log-level debug` 运行 `webots_bridge` 查看详细日志；确认 `robot_descriptions/<prefix>/worlds/<prefix>.wbt` 存在。 |

---

## 参考

- [Webots ROS 2 文档](https://github.com/cyberbotics/webots_ros2)
- [ros2_control](https://control.ros.org/)
- [TensorRT Deployment Guide](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html)
