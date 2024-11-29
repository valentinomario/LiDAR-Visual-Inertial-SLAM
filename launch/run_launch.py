from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    emv_lio_share = FindPackageShare("emv-lio2").find("emv-lio2")
    launch_dir = os.path.join(emv_lio_share, 'launch', 'include')

    return LaunchDescription([
        # Include sub-launch files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'module_robot_state_publisher_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'module_rviz_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'module_sam_hesai_launch.py'))
        ),
    ])
