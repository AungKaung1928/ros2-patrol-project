#!/usr/bin/env python3
"""
Combined Patrol Navigation Launch
Launches Gazebo, Nav2, and Patrol with proper timing
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with automatic delays"""
    
    # Get package directories
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    turtlebot3_navigation2_share = get_package_share_directory('turtlebot3_navigation2')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    package_share = get_package_share_directory('patrol_navigation_project')
    
    # Map and params
    default_map = os.path.join(turtlebot3_navigation2_share, 'map', 'map.yaml')
    default_params = os.path.join(nav2_bringup_share, 'params', 'nav2_params.yaml')
    
    # Arguments
    map_arg = DeclareLaunchArgument('map', default_value=default_map)
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    
    # Step 1: Launch Gazebo (starts immediately)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
        )
    )
    
    # Step 2: Launch Nav2 (after 30 second delay)
    nav2_launch = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )
    
    # Step 3: Launch RViz (after 30 second delay)
    rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    rviz_launch = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    # Step 4: Launch Patrol (after 45 second delay - gives Nav2 time to init)
    patrol_launch = TimerAction(
        period=45.0,
        actions=[
            Node(
                package='patrol_navigation_project',
                executable='patrol_controller',
                name='patrol_controller',
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        map_arg,
        params_file_arg,
        gazebo_launch,    # t=0s
        nav2_launch,      # t=30s
        rviz_launch,      # t=30s
        patrol_launch     # t=45s
    ])