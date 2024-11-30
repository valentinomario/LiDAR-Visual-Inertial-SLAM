from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description = PathJoinSubstitution([
        FindPackageShare("emv_lio2"),
        "launch",
        "include",
        "config",
        "robot.urdf.xacro"
    ])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", robot_description]),
            }],
            output="screen"
        )
    ])
