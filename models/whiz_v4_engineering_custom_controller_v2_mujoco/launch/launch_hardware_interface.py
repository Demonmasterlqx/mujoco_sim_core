import signal
from functools import partial

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    package_share = FindPackageShare("whiz_v4_engineering_custom_controller_v2_mujoco")
    model = PathJoinSubstitution([package_share, "whiz_v4_engineering_custom_controller_v2.xml"])
    urdf = PathJoinSubstitution([package_share, "config", "whiz_v4_engineering_custom_controller_v2.urdf.xacro"])
    controllers = PathJoinSubstitution([package_share, "config", "controllers.yaml"])
    robot_description = {"robot_description" : ParameterValue(Command([FindExecutable(name="xacro"), " ", urdf]), value_type=str)}
    
    
    server = ExecuteProcess(
        cmd=[
            FindExecutable(name="bash"),
            "-c",
            "trap '' INT; exec \"$@\"",
            "mujoco-server-wrapper",
            FindExecutable(name="mujoco-sim-server"),
            "--server-name",
            "whiz_v4_engineering_custom_controller_v2",
            "--model",
            model,
        ],
        output="screen",
    )

    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers],
        output="screen",
    )
    
    description_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    
    broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--activate-as-group",
        ],
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([package_share, "config", "rviz.rviz"])],
        output="screen",
    )
    
    return LaunchDescription(
        [
            server,
            control,
            description_publisher,
            broadcaster_spawner,
            rviz,
        ]
    )