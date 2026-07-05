import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Isaac Sim is the physics backend; ros2_control bridges to it via
    # joint_state_topic_hardware_interface:
    #   - publishes /isaac_joint_commands (sensor_msgs/JointState) to Isaac
    #   - subscribes /isaac_joint_states to read Isaac's simulated joint states
    # The rest (JTC controllers + move_group + rviz) is identical to mock/real.
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
            "use_isaac:=true",
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
    # topic delivery (which fails for late joiners under the custom FastDDS profile).
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

    # cuMotion (GPU) planner node for the LEFT arm. It advertises the MotionPlan
    # action server that the MoveIt "isaac_ros_cumotion" pipeline connects to.
    # Toggle with use_cumotion:=false to fall back to a pure-OMPL setup.
    use_cumotion = LaunchConfiguration("use_cumotion")
    declare_use_cumotion = DeclareLaunchArgument(
        "use_cumotion",
        default_value="true",
        description="Start the cuMotion planner node for the left arm.",
    )
    cumotion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("pkg_robot_model_cumotion_config"),
                    "launch",
                    "x1_cumotion.launch.py",
                ]
            )
        ),
        condition=IfCondition(use_cumotion),
    )

    delayed_spawners = TimerAction(period=3.0, actions=spawn_controllers)
    delayed_moveit = TimerAction(period=5.0, actions=[move_group_launch, moveit_rviz_launch])
    # Start cuMotion after move_group so the action server registers cleanly.
    delayed_cumotion = TimerAction(period=8.0, actions=[cumotion_launch])

    return LaunchDescription(
        [
            declare_use_cumotion,
            control_node,
            robot_state_pub_node,
            delayed_spawners,
            delayed_moveit,
            delayed_cumotion,
        ]
    )


