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


def _shutdown_on_failure(event, _context, *, process_name):
    if event.returncode == 0:
        return []
    return [
        EmitEvent(
            event=Shutdown(
                reason=f"{process_name} exited with return code {event.returncode}"
            )
        )
    ]


def _start_after_success(event, _context, *, action, delay, process_name):
    if event.returncode != 0:
        return _shutdown_on_failure(event, _context, process_name=process_name)
    return [TimerAction(period=delay, actions=[action])]


def generate_launch_description():
    package_share = FindPackageShare("mujoco_sim_hardware_interface_demo")
    model = PathJoinSubstitution([package_share, "mjcf", "diff_drive.xml"])
    urdf = PathJoinSubstitution([package_share, "urdf", "diff_drive.urdf"])
    controllers = PathJoinSubstitution([package_share, "config", "controllers.yaml"])
    robot_description = {"robot_description": Command([FindExecutable(name="cat"), " ", urdf])}

    auto_command_argument = DeclareLaunchArgument(
        "auto_command",
        default_value="true",
        description="Publish a bounded demonstration cmd_vel after the controller is ready",
    )
    command_duration_argument = DeclareLaunchArgument(
        "command_duration_s",
        default_value="3.0",
        description="Duration of the automatic demonstration command",
    )

    server = ExecuteProcess(
        cmd=[
            FindExecutable(name="bash"),
            "-c",
            "trap '' INT; exec \"$@\"",
            "mujoco-server-wrapper",
            FindExecutable(name="mujoco-sim-server"),
            "--headless",
            "--server-name",
            "ros2_control_demo",
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
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--activate-as-group",
        ],
    )
    start_broadcasters = TimerAction(
        period=2.0,
        actions=[broadcaster_spawner],
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )
    start_diff_drive_after_state = RegisterEventHandler(
        OnProcessExit(
            target_action=broadcaster_spawner,
            on_exit=partial(
                _start_after_success,
                action=diff_drive_spawner,
                delay=0.5,
                process_name="broadcaster spawner",
            ),
        )
    )
    stop_launch_on_diff_drive_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=partial(
                _shutdown_on_failure, process_name="diff-drive spawner"
            ),
        )
    )
    command_generator = Node(
        package="mujoco_sim_hardware_interface_demo",
        executable="command_generator.py",
        name="bounded_command_generator",
        condition=IfCondition(LaunchConfiguration("auto_command")),
        parameters=[
            {
                "duration_s": ParameterValue(
                    LaunchConfiguration("command_duration_s"), value_type=float
                )
            }
        ],
        output="screen",
    )
    stop_launch_on_command_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=command_generator,
            on_exit=partial(
                _shutdown_on_failure, process_name="bounded command generator"
            ),
        )
    )
    stop_server_after_control = RegisterEventHandler(
        OnProcessExit(
            target_action=control,
            on_exit=[
                EmitEvent(
                    event=SignalProcess(
                        signal_number=signal.SIGTERM,
                        process_matcher=matches_action(server),
                    )
                )
            ],
        )
    )
    stop_launch_after_server = RegisterEventHandler(
        OnProcessExit(
            target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason="MuJoCo Server exited"))],
        )
    )
    return LaunchDescription(
        [
            auto_command_argument,
            command_duration_argument,
            stop_server_after_control,
            stop_launch_after_server,
            start_diff_drive_after_state,
            stop_launch_on_diff_drive_failure,
            stop_launch_on_command_failure,
            server,
            description_publisher,
            control,
            start_broadcasters,
            command_generator,
        ]
    )
