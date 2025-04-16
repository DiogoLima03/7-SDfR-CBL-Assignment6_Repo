# For running the image_to_pose node in the VM without connecting to the motors (Xenomai).

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    
    image_to_pose = Node(
            package="image_to_pose",
            executable="image_to_pose",
            name="image_to_pose"
        )
    
    cam2image_windows_path = os.path.join(
        get_package_share_directory('launch_relbot'),
        'launch',
        'cam2image_windows_launch.py'
    )
    cam2image_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cam2image_windows_path)
        )
    
    return LaunchDescription([
        image_to_pose,
        cam2image_launch
    ])