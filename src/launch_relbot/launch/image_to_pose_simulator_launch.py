# For running the image_to_pose node in the VM without connecting to the motors (Xenomai).

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    
    config_image_to_pose = os.path.join(
        get_package_share_directory('launch_relbot'),
        'config',
        'image_to_pose_simulator.yaml')
    
    image_to_pose = Node(
            package="image_to_pose",
            executable="image_to_pose",
            name="image_to_pose",
            parameters=[config_image_to_pose],
        )
    
    config_cam2image = os.path.join(
        get_package_share_directory('launch_relbot'),
        'config',
        'cam2image_windows_server.yaml')
    
    cam2image_vm2ros = Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[config_cam2image],
            output='screen',
        )
    
    return LaunchDescription([
        image_to_pose,
        cam2image_vm2ros
    ])
    
    
    