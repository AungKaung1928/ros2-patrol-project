#!/usr/bin/env python3
"""
Patrol Visualizer - RViz marker publication for waypoints and patrol path
Provides real-time visualization of patrol state
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


class PatrolVisualizer(Node):
  """Publishes RViz markers for patrol waypoints and current target"""

  def __init__(self):
    super().__init__('patrol_visualizer')
    
    # Publishers
    self.marker_pub = self.create_publisher(
      MarkerArray,
      'patrol_waypoints',
      10
    )
    
    self.current_target_pub = self.create_publisher(
      Marker,
      'current_target',
      10
    )
    
    self.get_logger().info('Patrol visualizer initialized')

  def visualize_waypoints(self, waypoints):
    """
    Visualize all patrol waypoints with path line
    
    Args:
      waypoints: List of patrol point dictionaries
    """
    marker_array = MarkerArray()
    
    # Create individual waypoint markers
    for i, waypoint in enumerate(waypoints):
      marker_array.markers.append(
        self.create_waypoint_marker(waypoint, i)
      )
    
    # Create patrol path line
    marker_array.markers.append(
      self.create_path_marker(waypoints)
    )
    
    self.marker_pub.publish(marker_array)

  def visualize_current_target(self, current, index):
    """
    Visualize current target waypoint with arrow
    
    Args:
      current: Current patrol point dictionary
      index: Waypoint index
    """
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.ns = 'current_target'
    marker.id = 999
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    
    # Position
    marker.pose.position.x = current['x']
    marker.pose.position.y = current['y']
    marker.pose.position.z = current['z'] + 0.5
    
    # Orientation (pointing down)
    marker.pose.orientation.x = 0.707
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.707
    
    # Scale (large arrow)
    marker.scale.x = 0.5
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    
    # Color (bright yellow)
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    
    # Persist until deleted
    marker.lifetime = Duration(sec=0, nanosec=0)
    
    self.current_target_pub.publish(marker)
    
    self.get_logger().info(
      f"Visualizing target {index}: {current['name']}"
    )

  def create_waypoint_marker(self, point, index):
    """
    Create cylinder marker for waypoint
    
    Args:
      point: Patrol point dictionary
      index: Waypoint index
    
    Returns:
      Marker message
    """
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.ns = 'waypoints'
    marker.id = index
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    
    # Position
    marker.pose.position.x = point['x']
    marker.pose.position.y = point['y']
    marker.pose.position.z = 0.0
    
    # Orientation
    marker.pose.orientation.w = 1.0
    
    # Scale
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.1
    
    # Color (blue with transparency)
    marker.color.r = 0.0
    marker.color.g = 0.5
    marker.color.b = 1.0
    marker.color.a = 0.7
    
    marker.lifetime = Duration(sec=0, nanosec=0)
    
    return marker

  def create_path_marker(self, waypoints):
    """
    Create line strip marker for patrol path
    
    Args:
      waypoints: List of patrol point dictionaries
    
    Returns:
      Marker message
    """
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.ns = 'patrol_path'
    marker.id = 1000
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    
    # Create line through all waypoints
    for wp in waypoints:
      p = Point()
      p.x = wp['x']
      p.y = wp['y']
      p.z = 0.05  # Slightly above ground
      marker.points.append(p)
    
    # Close the loop
    if waypoints:
      p = Point()
      p.x = waypoints[0]['x']
      p.y = waypoints[0]['y']
      p.z = 0.05
      marker.points.append(p)
    
    # Line appearance
    marker.scale.x = 0.05  # Line width
    
    # Color (cyan)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 0.5
    
    marker.lifetime = Duration(sec=0, nanosec=0)
    
    return marker