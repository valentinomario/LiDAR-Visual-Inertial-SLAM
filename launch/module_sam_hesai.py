from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare("emv-lio2"), "config"])

    '''
    return LaunchDescription([
        # Lidar Odometry Parameters
        Node(
            package="emv-lio2",
            executable="emv-lio2_imuPreintegration",
            name="imuPreintegration",
            output="screen",
            parameters=[{
                "lio_config_file": PathJoinSubstitution([config_dir, "params_lidar.yaml"]),
                "vins_config_file": PathJoinSubstitution([config_dir, "params_camera.yaml"]),
                "vins_config_file_1": PathJoinSubstitution([config_dir, "params_camera1.yaml"]),
                "vins_config_file_2": PathJoinSubstitution([config_dir, "params_camera2.yaml"]),
            }]
        ),
        # Other Nodes
        Node(
            package="emv-lio2",
            executable="emv-lio2_imageProjection",
            name="imageProjection",
            output="screen"
        ),
        Node(
            package="emv-lio2",
            executable="emv-lio2_featureExtraction",
            name="featureExtraction",
            output="screen"
        ),
        Node(
            package="emv-lio2",
            executable="emv-lio2_mapOptmization",
            name="mapOptmization",
            output="screen"
        ),
        # Visual Odometry Nodes
        Node(
            package="emv-lio2",
            executable="emv-lio2_visual_feature",
            name="visual_feature",
            output="screen"
        ),
        Node(
            package="emv-lio2",
            executable="emv-lio2_visual_odometry",
            name="visual_odometry",
            output="screen"
        ),
        Node(
            package="emv-lio2",
            executable="emv-lio2_visual_loop",
            name="visual_loop",
            output="screen"
        ),
    ])
    '''

    return LaunchDescription([Node(
        package="emv-lio2",
        executable="emv-lio2_visual_feature",
        name="visual_feature",
        output="screen"
    )])