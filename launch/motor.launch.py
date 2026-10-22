from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("esp32_motor_hardware_interface")
    device = LaunchConfiguration("device")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                PathJoinSubstitution([package_share, "description", "urdf", "motor.urdf.xacro"]),
                " device:=", device,
            ]),
            value_type=str,
        )
    }
    controllers = PathJoinSubstitution([package_share, "config", "controllers.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/ttyTHS1",
                              description="Serial device the ESP32 is wired to"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controllers],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller"],
        ),
    ])
