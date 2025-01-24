import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gscam_pkg = 'gscam'
    gscam_launch = 'arducam.launch.xml'

    livox_pkg = 'livox_ros_driver2'
    livox_launch = 'msg_MID360_launch.py'

    gscam_launch_path = os.path.join(
        get_package_share_directory(gscam_pkg),
        'launch',
        gscam_launch
    )

    livox_launch_path = os.path.join(
        get_package_share_directory(livox_pkg),
        'launch_ROS2',
        livox_launch
    )

    print('gscam: '+ gscam_launch_path)
    print('livox: '+ livox_launch_path)

    return LaunchDescription([
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(gscam_launch_path),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(livox_launch_path)
            )
        ])