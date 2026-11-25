import signal
from functools import partial

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _continue_or_shutdown(event, _context, *, process_name, success_actions=()):
    if event.returncode == 0:
        return list(success_actions)
    return [
        EmitEvent(
            event=Shutdown(
                reason=f"{process_name} exited with return code {event.returncode}"
            )
        )
    ]


def generate_launch_description():
    package_share = FindPackageShare("whiz_v4_engineering_custom_controller_v2_mujoco")
    model = PathJoinSubstitution(
        [package_share, "whiz_v4_engineering_custom_controller_v2.xml"]
    )
    urdf = PathJoinSubstitution(
        [package_share, "config", "whiz_v4_engineering_custom_controller_v2.urdf.xacro"]
    )
    controllers = PathJoinSubstitution([package_share, "config", "controllers.yaml"])
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", urdf]), value_type=str
        )
    }

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
            "--initial-keyframe",
            "home",
            "--headless",
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
    controller_loader = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )
    initial_state_waiter = ExecuteProcess(
        cmd=[
            FindExecutable(name="timeout"),
            "--signal=TERM",
            "10s",
            FindExecutable(name="ros2"),
            "topic",
            "echo",
            "--once",
            "/clock",
            "rosgraph_msgs/msg/Clock",
        ],
        output="screen",
    )
    controller_activator = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "control",
            "switch_controllers",
            "--activate",
            "joint_state_broadcaster",
            "arm_controller",
            "--strict",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    start_controller_loader = TimerAction(period=2.0, actions=[controller_loader])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([package_share, "config", "rviz.rviz"])],
        output="screen",
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
    wait_for_initial_state_after_load = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_loader,
            on_exit=partial(
                _continue_or_shutdown,
                process_name="joint-state and arm-controller loader",
                success_actions=(initial_state_waiter,),
            ),
        )
    )
    activate_controllers_after_initial_state = RegisterEventHandler(
        OnProcessExit(
            target_action=initial_state_waiter,
            on_exit=partial(
                _continue_or_shutdown,
                process_name="initial hardware-state waiter",
                success_actions=(controller_activator,),
            ),
        )
    )
    stop_launch_on_activation_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_activator,
            on_exit=partial(
                _continue_or_shutdown,
                process_name="joint-state and arm-controller activator",
            ),
        )
    )

    return LaunchDescription(
        [
            stop_server_after_control,
            stop_launch_after_server,
            wait_for_initial_state_after_load,
            activate_controllers_after_initial_state,
            stop_launch_on_activation_failure,
            server,
            description_publisher,
            control,
            start_controller_loader,
            rviz,
        ]
    )
