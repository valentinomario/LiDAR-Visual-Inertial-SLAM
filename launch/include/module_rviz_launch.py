from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare("emv_lio2"),
        "launch",
        "include",
        "config",
        "rviz.rviz"
    ])
    print(rviz_config)
    return LaunchDescription([
        # Rviz Node
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            #arguments=["-d", rviz_config],
            output="screen"
        )
    ])
