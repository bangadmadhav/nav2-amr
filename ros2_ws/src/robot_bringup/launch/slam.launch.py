from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': '/scan_fixed',
            'autostart': True,
            'max_laser_range': 8.0,
        }]
    )

    twist_to_stamped_node = Node(
        package='helper_nodes',
        executable='twist_to_stamped',
        name='twist_to_stamped',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/cmd_vel',
            'output_topic': '/diff_drive_controller/cmd_vel'  # ← direct to controller
        }]
    )

    return LaunchDescription([
        slam_toolbox_node,
        twist_to_stamped_node
    ])