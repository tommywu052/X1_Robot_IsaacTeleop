from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # READ-ONLY verification: bring up ros2_control with the real hardware and
    # ONLY the joint_state_broadcaster. No arm/gripper trajectory controllers are
    # spawned and move_group is NOT started, so no motion command can be issued.
    # Use this to confirm serial comms: /joint_states should reflect the robot's
    # actual pose. (Torque is enabled on responding servos to HOLD current pose.)
    pkg_robot_model = FindPackageShare("pkg_robot_model")

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

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_pub_node,
            TimerAction(period=3.0, actions=[jsb]),
        ]
    )
