# For running the relbot sequence controller in the simulation environment (VM) with the turtlesim simulator.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
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
    
    config_sequence_controler = os.path.join(
        get_package_share_directory('launch_relbot'),
        'config',
        'node_relbot_sequence_controller_simulator.yaml')
    
    relbot_sequence_controller = Node(
            package="relbot_sequence_controller",
            executable="relbot_sequence_controller",
            name="relbot_sequence_controller",
            parameters=[config_sequence_controler]
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
    
    return LaunchDescription([
        relbot_simulator,
        relbot2turtlesim,
        turtlesim,
        relbot_sequence_controller,
        cam2image_vm2ros,
        image_to_pose
    ])