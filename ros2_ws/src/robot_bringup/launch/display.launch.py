from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Define package shares
    description_pkg_share = get_package_share_directory('robot_description')
    bringup_pkg_share = get_package_share_directory('robot_bringup')

    # Define various paths
    urdf_file = os.path.join(description_pkg_share, 'urdf', 'robot_description.urdf.xacro')
    rviz_config_file = os.path.join(bringup_pkg_share, 'config', 'newDisplayTest.rviz')

    # robot_description parameter
    robot_model_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Launch Argument for World Name
    world_name_arg = DeclareLaunchArgument(
        'world',
        default_value='world1',
        description='Name of the world file'
    )

    # Resolve world file path from argument
    world_file = PathJoinSubstitution([
        FindPackageShare('worlds'),
        LaunchConfiguration('world'),
        'world.sdf'  # appends .sdf automatically
    ])

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_model_description},
            {"use_sim_time": True}
        ]
    )

    # rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": [world_file, ' -r'] 
        }.items()
    )

    # Spawn robot into Gazebo
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "amr_bot",
            "-x", "0.0", "-y", "0.0", "-z", "0.1",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0"
        ],
        output="screen"
    )

    # Delay bridge to ensure sensor topics are available
    delayed_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{
                    "config_file": os.path.join(bringup_pkg_share, "config", "gazebo_bridge.yaml")
                }],
                remappings=[
                    ('/imu', '/imu/out') 
                ],
                output="screen"
            )
        ]
    )

    # Delay controller spawners slightly after bridge
    delayed_controllers = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager"
                ],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen',
                parameters=[{"use_sim_time": True}],
                remappings=[
                    ('cmd_vel', '/cmd_vel_stamped')
                ]
            )
        ]
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_robot_node]
    )

    odom_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=[
            '/diff_drive_controller/odom',
            '/odom'
        ],
        output='screen'
    )

    scan_frame_fixer_node = Node(
        package='helper_nodes',
        executable='scan_frame_fixer',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    twist_to_stamped_node = Node(
        package='helper_nodes',
        executable='twist_to_stamped',
        name='twist_to_stamped',
        parameters=[{
            'input_topic': '/cmd_vel_smoothed',
            'output_topic': '/diff_drive_controller/cmd_vel'  # ← direct to controller
        }]
    )
    # USE When running SLAM
    # twist_to_stamped_node = Node(
    #     package='helper_nodes',
    #     executable='twist_to_stamped',
    #     name='twist_to_stamped',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'input_topic': '/cmd_vel',
    #         'output_topic': '/diff_drive_controller/cmd_vel'  # ← direct to controller
    #     }]
    # )

    return LaunchDescription([
        world_name_arg,
        robot_state_publisher_node,
        rviz_node,
        gazebo_node,
        delayed_spawn,
        # spawn_robot_node,
        # bridge_node,
        # joint_state_broadcaster_spawner,
        # diff_drive_controller_spawner
        # delayed_ekf,
        delayed_bridge,
        delayed_controllers,
        odom_relay_node,
        scan_frame_fixer_node,
        twist_to_stamped_node
    ])