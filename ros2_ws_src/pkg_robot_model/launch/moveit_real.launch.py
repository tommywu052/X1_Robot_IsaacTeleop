import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Real robot mode: ros2_control loads the C++ XiaobeiHardwareInterface
    # plugin, which talks to the Feetech serial-bus servos over /dev/ttyACM0
    # at 1000000 baud (CH343 USB-serial, attached into WSL via usbipd).
    # Hardware branch is selected by: use_sim:=false use_isaac:=false use_mock:=false
    # Everything else (JTC controllers + move_group + RViz) is identical to the
    # Isaac/mock launches so MoveIt planning + execution behaves the same.
    pkg_robot_model = FindPackageShare("pkg_robot_model")
    pkg_moveit_config = FindPackageShare("pkg_robot_model_moveit_config")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([pkg_robot_model, "urdf", "pkg_robot_model.urdf.xacro"]),
            " ",
            "use_sim:=false",
            " ",
            "use_isaac:=false",
            " ",
            "use_mock:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [pkg_robot_model, "config", "my_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    controllers_to_spawn = [
        "joint_state_broadcaster",
        "right_arm_controller",
        "right_gripper_controller",
        "left_arm_controller",
        "left_gripper_controller",
        "head_controller",
    ]

    spawn_controllers = []
    for controller in controllers_to_spawn:
        spawn_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit_config, "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            "allow_trajectory_execution": "true",
            "moveit_manage_controllers": "true",
            "use_sim_time": "false",
        }.items(),
    )

    # RViz: pass model params directly so it does not depend on transient_local
    # topic delivery (same fix proven in moveit_isaac.launch.py).
    moveit_config = (
        MoveItConfigsBuilder(
            "pkg_robot_model", package_name="pkg_robot_model_moveit_config"
        ).to_moveit_configs()
    )
    rviz_config = PathJoinSubstitution(
        [pkg_moveit_config, "config", "moveit.rviz"]
    )
    moveit_rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    delayed_spawners = TimerAction(period=3.0, actions=spawn_controllers)
    delayed_moveit = TimerAction(period=5.0, actions=[move_group_launch, moveit_rviz_launch])

    return LaunchDescription(
        [
            control_node,
            robot_state_pub_node,
            delayed_spawners,
            delayed_moveit,
        ]
    )
