#!/usr/bin/env python3
"""
Waypoint Manager - YAML loading and waypoint sequence management
Handles patrol point configuration from YAML files
"""

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


class WaypointManager(Node):
  """Manages patrol waypoints loaded from YAML configuration"""

  def __init__(self):
    super().__init__('waypoint_manager')
    
    self.patrol_points = []
    self.load_patrol_points()

  def load_patrol_points(self):
    """Load patrol points from YAML configuration file"""
    try:
      # Get package share directory
      package_share_directory = get_package_share_directory(
        'patrol_navigation_project'
      )
      yaml_file = os.path.join(
        package_share_directory,
        'config',
        'patrol_points.yaml'
      )
      
      # Load YAML file
      with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)
      
      if 'patrol_points' in config:
        for point in config['patrol_points']:
          patrol_point = {
            'name': point['name'],
            'x': float(point['x']),
            'y': float(point['y']),
            'z': float(point['z'])
          }
          self.patrol_points.append(patrol_point)
      
      self.get_logger().info(
        f'Loaded {len(self.patrol_points)} patrol points'
      )
      
    except Exception as e:
      self.get_logger().error(f'Failed to load patrol points: {e}')
      # Default points if file loading fails
      self.patrol_points = [
        {'name': 'point_1', 'x': 2.0, 'y': 0.0, 'z': 0.0},
        {'name': 'point_2', 'x': 2.0, 'y': 2.0, 'z': 0.0},
        {'name': 'point_3', 'x': 0.0, 'y': 2.0, 'z': 0.0},
        {'name': 'point_4', 'x': 0.0, 'y': 0.0, 'z': 0.0}
      ]
      self.get_logger().warning('Using default patrol points')

  def get_patrol_points(self):
    """
    Get all patrol points
    
    Returns:
      List of patrol point dictionaries
    """
    return self.patrol_points

  def get_next_point(self, current_index):
    """
    Get next patrol point in sequence
    
    Args:
      current_index: Current waypoint index
    
    Returns:
      Tuple of (next_index, patrol_point)
    """
    next_index = (current_index + 1) % len(self.patrol_points)
    return (next_index, self.patrol_points[next_index])