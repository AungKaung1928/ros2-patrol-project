#!/usr/bin/env python3
"""
Patrol Gazebo Launch - Start TurtleBot3 Gazebo simulation
Launches the TurtleBot3 world for patrol navigation
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  """Generate launch description for Gazebo simulation"""
  
  # Get package directories
  turtlebot3_gazebo_share = FindPackageShare('turtlebot3_gazebo')
  
  # Include TurtleBot3 Gazebo launch
  gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      turtlebot3_gazebo_share, '/launch/turtlebot3_world.launch.py'
    ])
  )
  
  return LaunchDescription([
    gazebo_launch
  ])