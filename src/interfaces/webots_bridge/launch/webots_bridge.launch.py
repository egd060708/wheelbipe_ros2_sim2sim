#!/usr/bin/env python
import os
import launch
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    # 声名已经定义的启动参数
    prefix = LaunchConfiguration('prefix')
    yaml_path = LaunchConfiguration('yaml_path')
    urdf = LaunchConfiguration('urdf')
    ctrl_mode = LaunchConfiguration('ctrl_mode')
    
    # 获取参数（转为字符串使用）
    prefixS = prefix.perform(context)
    yaml_path_string = yaml_path.perform(context)
    
    # 启动webots，指明了wbt文件和ros2_supervisor参数
    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [FindPackageShare("robot_descriptions"), prefixS, "worlds", prefixS+".wbt"]
        ),
        ros2_supervisor=True,
    )
    # 设置触发事件，如果webots进程退出，则触发ros关闭事件
    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )
    # 声名机器人描述文件
    robot_description_content_dir = PathJoinSubstitution(
        [FindPackageShare("robot_descriptions"), prefixS , "xacro", urdf]
    )
    xacro_executable = FindExecutable(name="xacro")
    # 生成机器人描述，并传入参数，得到了机器人以及ros2_control的配置
    robot_description_content = Command(
        [
            PathJoinSubstitution([xacro_executable]),
            " ",
            robot_description_content_dir,
            " ",
            "ctrl_mode:=",
            ctrl_mode,
            " ",
            "sim_env:=",
            "webots",
        ]
    )
    # 机器人描述字典
    robot_description = {"robot_description": robot_description_content}
    
    robot_controllers = os.path.join(
        get_package_share_directory(yaml_path_string),
        "config",
        prefixS+".yaml",
    )
    
    robot_driver = WebotsController(
        robot_name=prefixS,
        parameters=[
            robot_description,
            robot_controllers,
            {"use_sim_time": True},
        ],
        respawn=True,
        namespace=prefixS,
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
                    {"frame_prefix": prefixS+"/"},
        ],
        namespace=prefixS,
    )
    
    return [
        webots,
        webots._supervisor,
        robot_state_pub_node,
        robot_driver,
        webots_event_handler
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="wheelbipe25_v3",
            description="Define robot prefix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="wbc",
            choices=["wbc", "sdk", "mcu"],
            description="Enable sdk of joint effort input",
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf",
            default_value="robot.xacro",
            description="Define urdf file in description folder",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "yaml_path",
            default_value="webots_bridge",
            description="Define yaml file folder",
        )
    )
    return LaunchDescription(
        declared_arguments  + 
        [OpaqueFunction(function=launch_setup)]
    )