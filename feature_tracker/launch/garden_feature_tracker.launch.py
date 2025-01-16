from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/garden/params_camera.yaml'
    ])


    vins_path = PathJoinSubstitution([
        config_pkg_path,
        'config/../'
    ])

    # Define the node
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        namespace='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
        }]
    )

    rviz_config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/vins_euroc_rviz.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # bafgile compression

    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='image_republish',
        output='screen',
        arguments=['compressed', 'in/compressed:=/camera/image_raw/compressed', 'raw', 'out:=/camera/image_raw'],
        respawn=True
    )
    return LaunchDescription([
        LogInfo(msg=['[feature tracker launch] config path: ', config_path]),
        feature_tracker_node,
        republish_node,
        rviz_node
    ])