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
    
    # Waypoint data
    self.patrol_points = []
    
    # Patrol behavior configuration (with defaults)
    self.loop_enabled = True
    self.wait_at_waypoint = 2.0
    self.retry_on_failure = True
    self.max_retries = 3
    self.timeout_per_waypoint = 60.0
    
    # Map metadata (optional)
    self.map_info = {}
    
    self.load_patrol_config()
  
  def load_patrol_config(self):
    """Load patrol configuration from YAML file"""
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
      
      # Parse waypoints
      if 'patrol_points' in config:
        for point in config['patrol_points']:
          patrol_point = {
            'name': point['name'],
            'x': float(point['x']),
            'y': float(point['y']),
            'z': float(point['z']),
            'description': point.get('description', '')  # Optional field
          }
          self.patrol_points.append(patrol_point)
      
      # Parse patrol behavior configuration
      if 'patrol_config' in config:
        patrol_cfg = config['patrol_config']
        self.loop_enabled = patrol_cfg.get('loop_enabled', True)
        self.wait_at_waypoint = float(patrol_cfg.get('wait_at_waypoint', 2.0))
        self.retry_on_failure = patrol_cfg.get('retry_on_failure', True)
        self.max_retries = int(patrol_cfg.get('max_retries', 3))
        self.timeout_per_waypoint = float(
          patrol_cfg.get('timeout_per_waypoint', 60.0)
        )
      
      # Parse optional map metadata
      if 'map_info' in config:
        self.map_info = config['map_info']
      
      # Log loaded configuration
      self.get_logger().info(
        f'Loaded {len(self.patrol_points)} patrol points'
      )
      self.get_logger().info(
        f'Config: loop={self.loop_enabled}, '
        f'wait={self.wait_at_waypoint}s, '
        f'retry={self.retry_on_failure}, '
        f'max_retries={self.max_retries}'
      )
      
    except Exception as e:
      self.get_logger().error(f'Failed to load patrol config: {e}')
      # Fallback to defaults
      self._load_default_points()
  
  def _load_default_points(self):
    """Load default patrol points if YAML loading fails"""
    self.patrol_points = [
      {'name': 'point_1', 'x': 2.0, 'y': 0.0, 'z': 0.0, 'description': ''},
      {'name': 'point_2', 'x': 2.0, 'y': 2.0, 'z': 0.0, 'description': ''},
      {'name': 'point_3', 'x': 0.0, 'y': 2.0, 'z': 0.0, 'description': ''},
      {'name': 'point_4', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'description': ''}
    ]
    self.get_logger().warning('Using default patrol points')
  
  def get_patrol_points(self):
    """
    Get all patrol points
    
    Returns:
      List of patrol point dictionaries
    """
    return self.patrol_points
  
  def get_patrol_config(self):
    """
    Get patrol behavior configuration
    
    Returns:
      Dictionary with patrol config parameters
    """
    return {
      'loop_enabled': self.loop_enabled,
      'wait_at_waypoint': self.wait_at_waypoint,
      'retry_on_failure': self.retry_on_failure,
      'max_retries': self.max_retries,
      'timeout_per_waypoint': self.timeout_per_waypoint
    }
  
  def get_next_point(self, current_index):
    """
    Get next patrol point in sequence
    
    Args:
      current_index: Current waypoint index
    
    Returns:
      Tuple of (next_index, patrol_point) or (None, None) if not looping
    """
    if not self.loop_enabled and current_index >= len(self.patrol_points) - 1:
      return (None, None)
    
    next_index = (current_index + 1) % len(self.patrol_points)
    return (next_index, self.patrol_points[next_index])
