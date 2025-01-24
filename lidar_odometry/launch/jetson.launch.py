from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    config_path = PathJoinSubstitution([
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

    # Define the feature tracker node
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

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        )


    return LaunchDescription([
        # LogInfo(msg=['[feature tracker launch] config path: ', config_path]),
        # LogInfo(msg=['[vins estimator launch] config path: ', config_path]),
        # LogInfo(msg=['[vins estimator launch] vins path: ', vins_path]),
        # LogInfo(msg=['[vins estimator launch] support path: ', support_path]),
        vins_estimator_node,
        #pose_graph_node,
        feature_tracker_node,
        rviz_node,
        robot_state_publisher
    ])
