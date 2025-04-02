import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('launch_relbot'),
        'config',
        'cam2image.yaml')
    
    cam2image_vm2ros = Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[config],
            output='screen',
        )
    
    return LaunchDescription([
        cam2image_vm2ros
    ])
    