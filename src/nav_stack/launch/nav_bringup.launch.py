#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('nav_stack')
    map_file = os.path.expanduser('~/tTEAM_11/src/map_server/my_nav_map.yaml')


    # ==============================================================================
    # 1. SHARED RESOURCES (Map Server & RViz)
    # ==============================================================================
    rviz_config_file = LaunchConfiguration('rviz_config')
    default_rviz_path = os.path.expanduser('~/tTEAM_11/rvizfolder/rvizmapfile.rviz')
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_path, 
        description='Full path to the RViz config file to use'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_file}]
    )

    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True, 
            'autostart': True, 
            'node_names': ['map_server']
        }]
    )

    # ==============================================================================
    # 2. OUR_BOT NAVIGATION STACK
    # ==============================================================================
    our_bot_amcl = Node(
        namespace='our_bot', package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_amcl.yaml')],
        remappings=[('map', '/map')]
    )
    our_bot_planner = Node(
        namespace='our_bot', package='nav2_planner', executable='planner_server', name='planner_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_planner_server.yaml')], 
        remappings=[('map', '/map'), ('global_costmap/map', '/map')]
    )
    our_bot_controller = Node(
        namespace='our_bot', package='nav2_controller', executable='controller_server', name='controller_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_controller.yaml')]
    )
    our_bot_behavior = Node(
        namespace='our_bot', package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_recovery.yaml')]
    )
    our_bot_navigator = Node(
        namespace='our_bot', package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'our_bot_bt_navigator.yaml'), 
                    {'default_nav_to_pose_bt_xml': os.path.join(pkg_share, 'config', 'our_bot_behavior.xml')}],
        remappings=[('goal_pose', '/our_bot/goal_pose')]
    )
    our_bot_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_our_bot', output='screen', 
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 
                     'node_names': ['our_bot/amcl', 'our_bot/planner_server', 'our_bot/controller_server', 'our_bot/behavior_server', 'our_bot/bt_navigator']}]
    )

    # ==============================================================================
    # 3. ENEMY_BOT NAVIGATION STACK
    # ==============================================================================
    enemy_bot_amcl = Node(
        namespace='enemy_bot', package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_amcl.yaml')],
        remappings=[('map', '/map')]
    )
    enemy_bot_planner = Node(
        namespace='enemy_bot', package='nav2_planner', executable='planner_server', name='planner_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_planner_server.yaml')], 
        remappings=[('map', '/map'), ('global_costmap/map', '/map')]
    )
    enemy_bot_controller = Node(
        namespace='enemy_bot', package='nav2_controller', executable='controller_server', name='controller_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_controller.yaml')]
    )
    enemy_bot_behavior = Node(
        namespace='enemy_bot', package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_recovery.yaml')]
    )
    enemy_bot_navigator = Node(
        namespace='enemy_bot', package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', 
        parameters=[os.path.join(pkg_share, 'config', 'enemy_bot_bt_navigator.yaml'), 
                    {'default_nav_to_pose_bt_xml': os.path.join(pkg_share, 'config', 'our_bot_behavior.xml')}],
        remappings=[('goal_pose', '/enemy_bot/goal_pose')]
    )
    enemy_bot_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_enemy_bot', output='screen', 
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 
                     'node_names': ['enemy_bot/amcl', 'enemy_bot/planner_server', 'enemy_bot/controller_server', 'enemy_bot/behavior_server', 'enemy_bot/bt_navigator']}]
    )

    # ==============================================================================
    # 4. COMPILE LAUNCH DESCRIPTION
    # ==============================================================================
    ld = LaunchDescription()

    # Shared
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(rviz_node)
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)

    # our_bot
    ld.add_action(our_bot_amcl)
    ld.add_action(our_bot_planner)
    ld.add_action(our_bot_controller)
    ld.add_action(our_bot_behavior)
    ld.add_action(our_bot_navigator)
    ld.add_action(our_bot_lifecycle)

    # enemy_bot
    ld.add_action(enemy_bot_amcl)
    ld.add_action(enemy_bot_planner)
    ld.add_action(enemy_bot_controller)
    ld.add_action(enemy_bot_behavior)
    ld.add_action(enemy_bot_navigator)
    ld.add_action(enemy_bot_lifecycle)

    return ld