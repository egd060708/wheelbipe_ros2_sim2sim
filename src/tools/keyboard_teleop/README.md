# Keyboard Teleop Package

键盘遥操作功能包，用于从终端读取键盘输入并发送运动命令到算法层。

## 功能说明

该节点订阅键盘输入，并将运动指令发布到以下话题：
- `motion_command` (geometry_msgs/Twist): 线速度和角速度指令
- `height_command` (std_msgs/Float64): 高度指令

## 编译

```bash
cd /home/lu/Git_Project/github/wheelbipe_ros2_sim2sim
colcon build --packages-select keyboard_teleop
source install/setup.bash
```

## 使用方法

### 直接运行节点（推荐）

由于该节点需要直接访问终端进行键盘输入，**强烈建议直接运行节点**，而不是通过 launch 文件启动：

```bash
# 使用默认参数（从默认配置文件读取）
ros2 run keyboard_teleop keyboard_teleop_node \
  --ros-args \
  --params-file $(ros2 pkg prefix keyboard_teleop)/share/keyboard_teleop/config/keyboard_teleop_params.yaml

# 使用自定义配置文件
ros2 run keyboard_teleop keyboard_teleop_node \
  --ros-args \
  --params-file /path/to/your/custom_params.yaml

# 使用命令行参数覆盖（包括命名空间前缀和模式）
ros2 run keyboard_teleop keyboard_teleop_node \
  --ros-args \
  --params-file $(ros2 pkg prefix keyboard_teleop)/share/keyboard_teleop/config/keyboard_teleop_params.yaml \
  -p prefix:="wheelbipe25_v3" \
  -p control_mode:=1 \
  -p linear_vel_rate:=0.8 \
  -p angular_vel_rate:=0.6
```

**重要说明：**
- 如果控制器在命名空间下运行（如 `/wheelbipe25_v3`），必须设置 `prefix` 参数以匹配控制器的命名空间
- `control_mode=0` 为步进模式（每次按键增加/减少固定值）
- `control_mode=1` 为连续模式（按键持续按下时按速率变化，松开时自动恢复到0）

### 使用 Launch 文件启动（不推荐）

**注意**：通过 launch 文件启动时，stdin 可能不是终端，导致无法读取键盘输入。如果必须使用 launch 文件，请确保在终端中直接运行 launch 命令：

```bash
# 使用默认配置文件启动
ros2 launch keyboard_teleop keyboard_teleop.launch.py

# 使用自定义配置文件启动
ros2 launch keyboard_teleop keyboard_teleop.launch.py \
  config_file:=/path/to/your/custom_params.yaml
```

## 键盘控制

### 运动控制
- **w/W**: 增加前进速度 (lin_vel_x)
- **s/S**: 减少前进速度 (lin_vel_x，后退)
- **q/Q**: 增加左侧速度 (lin_vel_y)
- **e/E**: 减少左侧速度 (lin_vel_y，向右)
- **a/A**: 增加角速度 (ang_vel_z，左转)
- **d/D**: 减少角速度 (ang_vel_z，右转)

### 高度控制
- **t/T**: 增加高度
- **g/G**: 减少高度

### 其他功能
- **空格键**: 停止所有运动（速度归零）
- **r/R**: 重置高度到默认值
- **x/X**: 退出程序

## 参数配置

所有参数都通过YAML配置文件进行管理。默认配置文件位于 `config/keyboard_teleop_params.yaml`。

### 控制模式

- `control_mode`: 控制模式
  - `0`: 步进模式（默认）- 每次按键增加/减少固定步进值
  - `1`: 连续模式 - 按键持续按下时按速率变化，松开时以相同速率恢复到0

### 主要参数

**通用参数：**
- `max_linear_vel`: 最大线速度 (m/s)
- `max_angular_vel`: 最大角速度 (rad/s)
- `min_height`: 最小高度 (m)
- `max_height`: 最大高度 (m)
- `default_height`: 初始默认高度 (m)
- `prefix`: 命名空间前缀（用于匹配控制器的命名空间，例如 "wheelbipe25_v3"）
- `motion_command_topic`: 运动指令话题名称
- `height_command_topic`: 高度指令话题名称

**模式0（步进模式）参数：**
- `linear_vel_step`: 线速度步进值 (m/s)
- `angular_vel_step`: 角速度步进值 (rad/s)
- `height_step`: 高度步进值 (m)

**模式1（连续模式）参数：**
- `linear_vel_rate`: 线速度变化速率 (m/s per second)
- `angular_vel_rate`: 角速度变化速率 (rad/s per second)

详细的参数说明和默认值请参考 `config/keyboard_teleop_params.yaml` 文件中的注释。

### 修改配置

1. **修改默认配置文件**：直接编辑 `src/tools/keyboard_teleop/config/keyboard_teleop_params.yaml`，然后重新编译
2. **使用自定义配置文件**：创建自己的配置文件，通过 launch 文件指定路径（见上方启动方法）

## 注意事项

1. **该节点必须直接访问终端才能读取键盘输入**，因此：
   - **强烈建议直接运行节点**（`ros2 run`），而不是通过 launch 文件启动
   - 如果通过 launch 文件启动，stdin 可能不是终端，导致无法读取键盘输入
   - 建议在独立的终端窗口中运行
2. 节点会自动恢复终端设置，但如果在异常退出时可能需要手动重置终端
3. 如果终端显示异常，可以运行 `reset` 命令恢复
4. 如果看到错误 "stdin is not a terminal!"，请使用 `ros2 run` 直接运行节点
