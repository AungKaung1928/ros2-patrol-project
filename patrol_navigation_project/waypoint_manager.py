#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        self.patrol_points = []
        self.load_patrol_points()
        
    def load_patrol_points(self):
        """Load patrol points from YAML file"""
        try:
            package_share_directory = get_package_share_directory('patrol_navigation_project')
            yaml_file = os.path.join(package_share_directory, 'config', 'patrol_points.yaml')
            
            with open(yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                self.patrol_points = data['patrol_points']
                
            self.get_logger().info(f'Loaded {len(self.patrol_points)} patrol points')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load patrol points: {e}')
            # Default points if file loading fails
            self.patrol_points = [
                {'name': 'point_1', 'x': 2.0, 'y': 0.0, 'z': 0.0},
                {'name': 'point_2', 'x': 2.0, 'y': 2.0, 'z': 0.0},
                {'name': 'point_3', 'x': 0.0, 'y': 2.0, 'z': 0.0},
                {'name': 'point_4', 'x': 0.0, 'y': 0.0, 'z': 0.0}
            ]
    
    def get_patrol_points(self):
        """Return list of patrol points"""
        return self.patrol_points
    
    def get_next_point(self, current_index):
        """Get next patrol point in sequence"""
        next_index = (current_index + 1) % len(self.patrol_points)
        return next_index, self.patrol_points[next_index]

def main(args=None):
    rclpy.init(args=args)
    waypoint_manager = WaypointManager()
    
    try:
        rclpy.spin(waypoint_manager)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()