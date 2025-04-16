# For running the relbot fully, with motor (Xenomai).

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    
    relbot_sequence_controller = Node(
            package="relbot_sequence_controller",
            executable="relbot_sequence_controller",
            name="relbot_sequence_controller"
        )
    
    image_to_pose = Node(
            package="image_to_pose",
            executable="image_to_pose",
            name="image_to_pose"
        )
    
    showimage = Node(
            package="image_tools",
            executable="showimage",
            name="showimage"
        )
    
    cam2image_relbot_path = os.path.join(
        get_package_share_directory('launch_relbot'),
        'launch',
        'cam2image_relbot_launch.py'
    )
    cam2image_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cam2image_relbot_path)
        )
    
    return LaunchDescription([
        relbot_sequence_controller,
        image_to_pose,
        showimage,
        cam2image_launch
    ])