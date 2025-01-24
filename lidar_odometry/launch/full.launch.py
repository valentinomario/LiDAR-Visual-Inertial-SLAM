from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    camera_config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/jetson/params_camera.yaml'
    ])

    xacro_path = PathJoinSubstitution([
        config_pkg_path,
        'config/jetson/robot.urdf.xacro'
    ])

    vins_path = PathJoinSubstitution([
        config_pkg_path,
        'config/../'
    ])

    support_path = PathJoinSubstitution([
        config_pkg_path,
        'support_files'
    ])

    rviz_config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/jetson/jetson_full.rviz'
    ])

    # VINS

    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker_exec',
        name='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': camera_config_path,
            'vins_folder': vins_path
        }]
    )

    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator_exec',
        name='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': camera_config_path,
            'vins_folder': vins_path
        }]
    )

    # LIO

    lidar_config_path = PathJoinSubstitution([
        config_pkg_path,
        'config',
        'jetson',
        'params_lidar.yaml'
    ])


    imu_preintegration = Node(
        package='emv_lio2',
        executable='emv_lio2_imuPreintegration',
        name='emv_lio2_imuPreintegration',
        parameters=[lidar_config_path],
        output='screen'
    )

    image_projection = Node(
        package='emv_lio2',
        executable='emv_lio2_imageProjection',
        name='emv_lio2_imageProjection',
        parameters=[lidar_config_path],
        output='screen'
    )

    lio_feature_extraction = Node(
       package='emv_lio2',
       executable='emv_lio2_featureExtraction',
       name='emv_lio2_featureExtraction',
       parameters=[lidar_config_path],
       output='screen'
    )
    map_optimization = Node(
        package='emv_lio2',
        executable='emv_lio2_mapOptimization',
        name='emv_lio2_mapOptimization',
        parameters=[lidar_config_path],
        output='screen'
    )


    # Common

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_path])
        }]
    )

    odom_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        parameters=[lidar_config_path],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        vins_estimator_node,
        feature_tracker_node,
        imu_preintegration,
        image_projection,
        lio_feature_extraction,
        map_optimization,
        robot_state_publisher,
        odom_transform_publisher,
        rviz_node
    ])



