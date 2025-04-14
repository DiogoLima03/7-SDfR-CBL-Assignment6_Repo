import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    cam2image_path = os.path.join(
        get_package_share_directory('launch_relbot'),
        'launch',
        'cam2image_launch.py'
    )
    
    relbot_simulator = Node(
            package="relbot_simulator",
            executable="relbot_simulator",
            name="relbot_simulator"
        )
    
    relbot2turtlesim = Node(
            package="relbot2turtlesim",
            executable="relbot2turtlesim",
            name="relbot2turtlesim"
        )
    
    turtlesim = Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim"
        )
    
    relbot_sequence_controller = Node(
            package="relbot_sequence_controller",
            executable="relbot_sequence_controller",
            name="relbot_sequence_controller"
        )
    
    cam2image_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cam2image_path)
        )
    
    image_to_pose = Node(
            package="image_to_pose",
            executable="image_to_pose",
            name="image_to_pose"
        )
    
    return LaunchDescription([
        relbot_simulator,
        relbot2turtlesim,
        turtlesim,
        relbot_sequence_controller,
        cam2image_launch,
        image_to_pose
    ])