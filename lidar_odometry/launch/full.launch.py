from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/garden/params_camera.yaml'
    ])

    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            config_pkg_path, 'config', 'params_lidar.yaml']),
        description='FPath to the ROS2 parameters file to use.')

    xacro_path = PathJoinSubstitution([
        config_pkg_path,
        'config/garden/robot.urdf.xacro'
    ])

    vins_path = PathJoinSubstitution([
        config_pkg_path,
        'config/../'
    ])

    support_path = PathJoinSubstitution([
        config_pkg_path,
        'support_files'
    ])

    # Define the node
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker_exec',
        name='feature_tracker',
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

    # Define the vins_estimator node
    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator_exec',
        name='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
        }]
    )

    # Define the pose_graph node
    pose_graph_node = Node(
        package='pose_graph',
        executable='pose_graph',
        name='pose_graph',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'support_file': support_path,
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0
        }]
    )

    imu_preintegration = Node(
            package='emv_lio2',
            executable='emv_lio2_imuPreintegration',
            name='emv_lio2_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        )
    image_projection = Node(
            package='emv_lio2',
            executable='emv_lio2_imageProjection',
            name='emv_lio2_imageProjection',
            parameters=[parameter_file],
            output='screen'
        )
    feature_extraction = Node(
            package='emv_lio2',
            executable='emv_lio2_featureExtraction',
            name='emv_lio2_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        )
    map_optimization = Node(
            package='emv_lio2',
            executable='emv_lio2_mapOptimization',
            name='emv_lio2_mapOptimization',
            parameters=[parameter_file],
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

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        )
    # static map transform
    static_map = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
        )

    return LaunchDescription([
        params_declare,
        # LogInfo(msg=['[feature tracker launch] config path: ', config_path]),
        # LogInfo(msg=['[vins estimator launch] config path: ', config_path]),
        # LogInfo(msg=['[vins estimator launch] vins path: ', vins_path]),
        # LogInfo(msg=['[vins estimator launch] support path: ', support_path]),
        # VINS
        vins_estimator_node,
        #pose_graph_node,
        feature_tracker_node,
        rviz_node,
        republish_node,
        
        # LIO
        imu_preintegration,
        image_projection,
        feature_extraction,
        map_optimization,
        
        robot_state_publisher,
        static_map
    ])
