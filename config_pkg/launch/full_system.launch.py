import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lio = 'lidar_odometry'
    vins = 'vins_estimator'

    launch = 'run.launch.py'

    lio_launch_path = os.path.join(
        get_package_share_directory(lio),
        'launch',
        launch
    )

    vins_launch_path = os.path.join(
        get_package_share_directory(vins),
        'launch',
        launch
    )


    return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lio_launch_path),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vins_launch_path)
            )
        ])