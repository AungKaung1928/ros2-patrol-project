#!/usr/bin/env python3
"""
Patrol Controller - Main navigation orchestration node
Manages autonomous patrol between waypoints using Nav2 action client
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from .waypoint_manager import WaypointManager
from .patrol_visualizer import PatrolVisualizer
import time
import math


class PatrolController(Node):
  """Main patrol controller node for autonomous waypoint navigation"""

  def __init__(self):
    super().__init__('patrol_controller')
    
    # State variables
    self.current_point_index = 0
    self.is_patrolling = False
    self.goal_handle = None
    
    # Step 1: Initialize action client for navigation system
    self.navigate_to_pose_client = ActionClient(
      self,
      NavigateToPose,
      'navigate_to_pose'
    )
    
    # Step 2: Initialize initial pose publisher
    self.initial_pose_pub = self.create_publisher(
      PoseWithCovarianceStamped,
      'initialpose',
      10
    )
    
    # Step 3: Initialize waypoint manager and load patrol points
    self.waypoint_manager = WaypointManager()
    self.patrol_points = self.waypoint_manager.get_patrol_points()
    
    # Initialize visualizer
    self.visualizer = PatrolVisualizer()
    
    self.get_logger().info('Patrol controller with visualization initialized')
    
    # Step 4: Wait for Nav2 to be ready
    self.get_logger().info('Waiting for Nav2 to be ready...')
    self.wait_for_nav2()
    self.get_logger().info('Nav2 is ready!')
    
    # Start patrol
    self.start_patrol()

  def wait_for_nav2(self):
    """Wait for Nav2 action server to be available and initialize"""
    # Wait for the action server to be available
    while not self.navigate_to_pose_client.wait_for_server(timeout_sec=5.0):
      self.get_logger().info(
        'Waiting for navigate_to_pose action server to be available...'
      )
    
    # Additional wait to ensure Nav2 is fully ready
    self.get_logger().info('Waiting for Nav2 to fully initialize...')
    time.sleep(10.0)  # Give Nav2 time to fully start
    
    # Set initial pose (at origin)
    self.set_initial_pose()
    
    # Wait for navigation stack to be fully ready after initial pose
    self.get_logger().info('Waiting for navigation stack to activate...')
    time.sleep(5.0)  # Additional wait after setting initial pose

  def create_pose_stamped(self, x, y, z=0.0, yaw=0.0):
    """
    Create a PoseStamped message from coordinates
    
    Args:
      x: X coordinate in meters
      y: Y coordinate in meters
      z: Z coordinate in meters (default 0.0)
      yaw: Orientation around Z-axis in radians (default 0.0)
    
    Returns:
      PoseStamped message
    """
    pose = PoseStamped()
    
    # Set reference frame and timestamp
    pose.header.frame_id = 'map'
    pose.header.stamp = self.get_clock().now().to_msg()
    
    # Set 3D position coordinates
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    
    # Convert yaw to quaternion
    # For 2D rotation around Z-axis: q = [0, 0, sin(yaw/2), cos(yaw/2)]
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    return pose

  def set_initial_pose(self):
    """Step 5: Publish initial pose to set robot location at origin"""
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = self.get_clock().now().to_msg()
    
    # Set position at origin (0, 0, 0)
    initial_pose.pose.pose.position.x = 0.0
    initial_pose.pose.pose.position.y = 0.0
    initial_pose.pose.pose.position.z = 0.0
    
    # Set orientation (facing forward)
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0
    
    # Set covariance (small values for high confidence)
    initial_pose.pose.covariance = [0.0] * 36
    initial_pose.pose.covariance[0] = 0.08   # x variance
    initial_pose.pose.covariance[7] = 0.08   # y variance
    initial_pose.pose.covariance[35] = 0.05  # yaw variance
    
    # Publish initial pose
    self.initial_pose_pub.publish(initial_pose)
    self.get_logger().info('Published initial pose at origin')
    
    # Wait for AMCL to process the initial pose
    time.sleep(2.0)

  def start_patrol(self):
    """Step 6: Start autonomous patrol through waypoints"""
    self.get_logger().info(
      f'Starting patrol with {len(self.patrol_points)} waypoints...'
    )
    self.is_patrolling = True
    
    # Visualize all waypoints
    self.visualizer.visualize_waypoints(self.patrol_points)
    
    self.go_to_next_point()

  def go_to_next_point(self):
    """Step 7: Navigate to next patrol point"""
    if not self.patrol_points:
      self.get_logger().error('No patrol points available!')
      return
    
    current_point = self.patrol_points[self.current_point_index]
    self.get_logger().info(
      f"Going to {current_point['name']} at "
      f"({current_point['x']:.2f}, {current_point['y']:.2f})"
    )
    
    # Visualize current target
    self.visualizer.visualize_current_target(
      current_point,
      self.current_point_index
    )
    
    # Create goal pose
    goal_pose = self.create_pose_stamped(
      current_point['x'],
      current_point['y'],
      current_point['z']
    )
    
    # Step 8: Create goal message
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = goal_pose
    
    # Step 9: Send goal with callbacks
    self.get_logger().info('Sending navigation goal...')
    send_goal_future = self.navigate_to_pose_client.send_goal_async(
      goal_msg,
      feedback_callback=self.feedback_callback
    )
    send_goal_future.add_done_callback(self.goal_response_callback)

  def goal_response_callback(self, future):
    """Step 10: Handle goal acceptance/rejection"""
    goal_handle = future.result()
    
    if not goal_handle.accepted:
      self.get_logger().error('Goal was rejected by server')
      return
    
    self.get_logger().info('Goal accepted by server, waiting for result')
    self.goal_handle = goal_handle
    
    # Get result
    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(self.result_callback)

  def feedback_callback(self, feedback_msg):
    """Step 11: Handle navigation feedback (optional)"""
    # Can log progress here if needed
    pass

  def result_callback(self, future):
    """Step 12: Handle navigation result"""
    result = future.result()
    status = result.status
    
    # Step 12-16: Handle different result codes
    if status == 4:  # SUCCEEDED
      current_point = self.patrol_points[self.current_point_index]
      self.get_logger().info(f"Reached {current_point['name']}!")
      
      # Wait at patrol point
      time.sleep(2.0)
      
      # Step 13: Move to next point (circular patrol)
      self.current_point_index = (
        (self.current_point_index + 1) % len(self.patrol_points)
      )
      
      # Continue patrol if still active
      if self.is_patrolling:
        self.go_to_next_point()
    
    elif status == 5:  # CANCELED (Step 14)
      self.get_logger().info('Navigation was canceled')
    
    elif status == 6:  # ABORTED (Step 15)
      self.get_logger().error('Navigation failed! Retrying...')
      time.sleep(2.0)
      if self.is_patrolling:
        self.go_to_next_point()  # Step 16
    
    else:
      self.get_logger().error(f'Unknown result code: {status}')

  def stop_patrol(self):
    """Stop autonomous patrol"""
    self.get_logger().info('Stopping patrol...')
    self.is_patrolling = False
    
    if self.goal_handle:
      self.goal_handle.cancel_goal_async()


def main(args=None):
  """Main entry point"""
  rclpy.init(args=args)
  
  patrol_controller = PatrolController()
  
  try:
    rclpy.spin(patrol_controller)
  except KeyboardInterrupt:
    patrol_controller.get_logger().info('Shutting down patrol controller...')
    patrol_controller.stop_patrol()
  except Exception as e:
    patrol_controller.get_logger().error(f'Exception in patrol_controller: {e}')
    patrol_controller.stop_patrol()
  finally:
    patrol_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()