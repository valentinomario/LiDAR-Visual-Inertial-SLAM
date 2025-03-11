import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('config_pkg')

    lio_config_file = os.path.join(share_dir, 'launch', 'include', 'lidar_odometry', 'rviz.rviz')
    vins_config_file = os.path.join(share_dir, 'launch', 'include', 'visual_odometry', 'rviz.rviz')

    return LaunchDescription([
       
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', lio_config_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', vins_config_file],
            output='screen'
        )
    ])
