#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import os

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        self.waypoint_file = os.path.expanduser('~/waypoints.yaml')
        self.waypoints = {}
        self.count = 1
        
        self.get_logger().info("Waypoint logger started! Use the 'Publish Point' tool in RViz to click on the map.")
        self.get_logger().info(f"Waypoints will be saved to {self.waypoint_file}")

    def clicked_point_callback(self, msg):
        name = f"waypoint_{self.count}"
        
        self.waypoints[name] = {
            'position': {
                'x': msg.point.x,
                'y': msg.point.y,
                'z': 0.0
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0
            }
        }
        
        with open(self.waypoint_file, 'w') as f:
            yaml.dump(self.waypoints, f, default_flow_style=False)
            
        self.get_logger().info(f"Saved {name}: x={msg.point.x:.2f}, y={msg.point.y:.2f}")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    logger = WaypointLogger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
