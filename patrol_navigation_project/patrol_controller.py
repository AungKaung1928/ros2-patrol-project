#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
from .waypoint_manager import WaypointManager
from math import sin, cos

class PatrolController(Node):
    def __init__(self):
        super().__init__('patrol_controller')
        
        # Initialize navigator
        self.navigator = BasicNavigator()
        
        # Initialize waypoint manager
        self.waypoint_manager = WaypointManager()
        self.patrol_points = self.waypoint_manager.get_patrol_points()
        
        # Patrol state
        self.current_point_index = 0
        self.is_patrolling = False
        
        # Wait for Nav2 to be ready
        self.get_logger().info('Waiting for Nav2 to be ready...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready!')
        
        # Starting patrol
        self.start_patrol()
        
    def create_pose_stamped(self, x, y, z=0.0, yaw=0.0):
        """Create a PoseStamped message with proper yaw handling"""
        pose = PoseStamped()
    
        # Set reference frame and timestamp
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
    
        # Set 3D position coordinates
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
    
        # Convert yaw to quaternion (FIXED VERSION)
        # For 2D rotation around Z-axis: q = [0, 0, sin(yaw/2), cos(yaw/2)]
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)
        
        return pose
    
    def start_patrol(self):
        """Start the patrol sequence"""
        self.get_logger().info('Starting patrol...')
        self.is_patrolling = True
        self.go_to_next_point()
    
    def go_to_next_point(self):
        """Navigate to the next patrol point"""
        if not self.patrol_points:
            self.get_logger().error('No patrol points available!')
            return
        
        current_point = self.patrol_points[self.current_point_index]
        self.get_logger().info(f'Going to {current_point["name"]} at ({current_point["x"]}, {current_point["y"]})')
        
        # Create goal pose
        goal_pose = self.create_pose_stamped(
            current_point['x'], 
            current_point['y'], 
            current_point['z']
        )
        
        # Send goal to navigator
        self.navigator.goToPose(goal_pose)
        
        # Start checking navigation result
        self.check_navigation_result()
    
    def check_navigation_result(self):
        """Check if navigation to current point is complete"""
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            current_point = self.patrol_points[self.current_point_index]
            self.get_logger().info(f'Reached {current_point["name"]}!')
            
            # Wait a bit at the patrol point
            time.sleep(2.0)
            
            # Move to next point
            self.current_point_index = (self.current_point_index + 1) % len(self.patrol_points)
            
            # Continue patrol if still active
            if self.is_patrolling:
                self.go_to_next_point()
                
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation was canceled')
            
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed! Retrying...')
            time.sleep(2.0)
            if self.is_patrolling:
                self.go_to_next_point()
    
    def stop_patrol(self):
        """Stop the patrol"""
        self.get_logger().info('Stopping patrol...')
        self.is_patrolling = False
        self.navigator.cancelTask()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        patrol_controller = PatrolController()
        rclpy.spin(patrol_controller)
    except KeyboardInterrupt:
        patrol_controller.stop_patrol()
    finally:
        patrol_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
