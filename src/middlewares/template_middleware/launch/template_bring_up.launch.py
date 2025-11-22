import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix').perform(context)
    urdf = "robot_sim.xacro"
    yaml_path = "template_middleware"
    
    # launch webots bridge
    webots_controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('webots_bridge'),
            'launch',
            'webots_bridge.launch.py'
        )),
        launch_arguments={
            'prefix': LaunchConfiguration('prefix'),
            'ctrl_mode': LaunchConfiguration('ctrl_mode'),
            'urdf': urdf,
            'yaml_path': yaml_path,
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('sim_env'), "' == 'webots'"]))
    )
    
    # launch ros2ctrl middleware
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )
    
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )
    
    # launch ros2 controllers
    robot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["template_ros2_controller", "--controller-manager", prefix+"/controller_manager"],
    )
    
    return [
        webots_controller_manager_launch,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        robot_controller,
    ]


def generate_launch_description():
    # Declare an argument to control inclusion of the extra launch file
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
            "sim_env",
            default_value="webots",
            description="Select simulation environment",
            choices=["webots", "gazebo"]
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="wbc",
            choices=["wbc", "sdk", "mcu"],
            description="Select wheel-legged robot control methods, mcu means on mcu-board control",
        )
    )
    
    
    return LaunchDescription(declared_arguments + 
                             [OpaqueFunction(function=launch_setup)])
