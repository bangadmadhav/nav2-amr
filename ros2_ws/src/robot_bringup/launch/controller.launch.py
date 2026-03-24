from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.5',
        description='Radius of the robot wheels'
    )
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.306',
        description='Distance between the robot wheels'
    )

    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')


    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,   
    ])