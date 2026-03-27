from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

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
            'autostart': True,        #Trying this so to avoid lifecycle configure and activate commands

        }]
    )

    scan_frame_fixer_node = Node(
        package='testing',
        executable='scan_frame_fixer',
        name='scan_frame_fixer',
        output='screen'
    )

    twist_to_stamped_node = Node(
        package='testing',
        executable='twist_to_stamped',
        name='twist_to_stamped',
        output='screen'
    )

    return LaunchDescription([
        scan_frame_fixer_node,
        twist_to_stamped_node,
        slam_toolbox_node,
    ])