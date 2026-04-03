import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    bringup_pkg = get_package_share_directory('robot_bringup')
    nav2_config_dir = os.path.join(bringup_pkg, 'config', 'nav2')  

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='world1_map',
        description='Map yaml filename without extension'
    )

    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='navfn',
        description='navfn | theta_star | smac'
    )

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='rpp',
        description='dwb | rpp | graceful | mppi'
    )

    map_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'maps',
        [LaunchConfiguration('map_name'), '.yaml']
    ])

    # Parameter Files
    amcl_params = os.path.join(bringup_pkg, 'config', 'nav2', 'amcl_params.yaml')
    costmap_params = os.path.join(bringup_pkg, 'config', 'nav2', 'costmap_params.yaml')
    bt_params = os.path.join(nav2_config_dir, 'bt_navigator_params.yaml')
    behavior_params = os.path.join(nav2_config_dir, 'behavior_params.yaml')

    planner_params = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config',
        'nav2',
        'planners',
        [LaunchConfiguration('planner'), '_params.yaml']
    ])
    
    controller_params = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'nav2', 'controllers',
        [LaunchConfiguration('controller'), '_params.yaml']
    ])

    # ── Map Server ────────────────────────────────────────────────────────────
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file
        }]
    )

    # ── AMCL ──────────────────────────────────────────────────────────────────
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params]
    )

    # ── Planner Server (contains global costmap) ──────────────────────────────
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[costmap_params, planner_params]
    )

    # ── Controller Server ──────────────────────────────
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[costmap_params, controller_params],
        remappings=[
            ('cmd_vel', '/cmd_vel')
        ]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[costmap_params, behavior_params]
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[behavior_params],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed')
        ]
    )
    
    # ── Behavior Tree Navigator ───────────────────────────────────────────────
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_params]
    )

    # ── Lifecycle Manager ─────────────────────────────────────
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server', 
                'amcl', 
                'planner_server',
                'controller_server',
                'behavior_server',
                'velocity_smoother',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        map_name_arg,
        planner_arg,
        controller_arg,

        map_server_node,
        amcl_node,

        planner_server_node,
        controller_server_node,
        behavior_server_node,
        velocity_smoother_node,

        bt_navigator_node,

        lifecycle_manager_node,
    ])