import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ������C�� MoveIt �yԇ���� ros2_control �� mock_components/GenericSystem
    # ȡ�� C++ Ӳ�w���棬position ָ���ֱ��ޒ�ڳ� position ��B��
    # �����c real_robot.launch.py ��ȫһ�£�ֻ����Ӳ�w���Q�� mock��
    pkg_robot_model = FindPackageShare("pkg_robot_model")
    pkg_moveit_config = FindPackageShare("pkg_robot_model_moveit_config")

    # ���� URDF��use_sim:=false �P�� gazebo��use_mock:=true ���ü�Ӳ�w
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([pkg_robot_model, "urdf", "pkg_robot_model.urdf.xacro"]),
            " ",
            "use_sim:=false",
            " ",
            "use_mock:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [pkg_robot_model, "config", "my_controllers.yaml"]
    )

    # controller_manager�����d�� mock Ӳ�w��
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # TF �l��
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

    # MoveIt ���X + RViz��use_sim_time:=false�����]�� gazebo �r犣�
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

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit_config, "launch", "moveit_rviz.launch.py"])
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
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
