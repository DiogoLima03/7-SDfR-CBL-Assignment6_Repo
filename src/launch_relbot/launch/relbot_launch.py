# For running the relbot fully, with motor (Xenomai).

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_sequence_controler = os.path.join(
        get_package_share_directory('launch_relbot'),
        'config',
        'node_relbot_sequence_controller_relbot.yaml')
    
    relbot_sequence_controller = Node(
            package="relbot_sequence_controller",
            executable="relbot_sequence_controller",
            name="relbot_sequence_controller",
            parameters=[config_sequence_controler]
        )
    
    config_image_to_pose = os.path.join(
        get_package_share_directory('launch_relbot'),
        'config',
        'image_to_pose_relbot.yaml')
    
    image_to_pose = Node(
            package="image_to_pose",
            executable="image_to_pose",
            name="image_to_pose",
            parameters=[config_image_to_pose],
        )
    
    config_cam2image = os.path.join(
        get_package_share_directory('launch_relbot'),
        'config',
        'cam2image_relbot.yaml')
    
    cam2image_vm2ros = Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[config_cam2image],
            output='screen',
        )
    
    return LaunchDescription([
        relbot_sequence_controller,
        image_to_pose,
        cam2image_vm2ros
    ])