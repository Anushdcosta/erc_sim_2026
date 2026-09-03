#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml
import os

class SavePoseNode(Node):
    def __init__(self):
        super().__init__('save_pose_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoint_file = os.path.expanduser('~/waypoints.yaml')
        
        # Load existing waypoints if any
        self.waypoints = {}
        if os.path.exists(self.waypoint_file):
            with open(self.waypoint_file, 'r') as f:
                loaded = yaml.safe_load(f)
                if loaded:
                    self.waypoints = loaded

    def save_current_pose(self, name):
        try:
            # Get the transform from map to base_link
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
                
            self.waypoints[name] = {
                'position': {
                    'x': t.transform.translation.x,
                    'y': t.transform.translation.y,
                    'z': t.transform.translation.z
                },
                'orientation': {
                    'x': t.transform.rotation.x,
                    'y': t.transform.rotation.y,
                    'z': t.transform.rotation.z,
                    'w': t.transform.rotation.w
                }
            }
            
            with open(self.waypoint_file, 'w') as f:
                yaml.dump(self.waypoints, f, default_flow_style=False)
                
            print(f"\n[SUCCESS] Saved current robot pose as '{name}'")
            print(f"X: {t.transform.translation.x:.2f}, Y: {t.transform.translation.y:.2f}")
            
        except TransformException as ex:
            print(f"\n[ERROR] Could not get robot pose: {ex}")

def main():
    rclpy.init()
    node = SavePoseNode()
    
    print("=========================================")
    print("Robot Pose Saver")
    print("Drive the robot using teleop to a location.")
    print("Type a name (e.g. 'shelf_1') and press Enter to save.")
    print("Type 'exit' to quit.")
    print("=========================================")
    
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    while True:
        try:
            name = input("\nEnter waypoint name to save current pose: ").strip()
            if name.lower() == 'exit':
                break
            if name:
                node.save_current_pose(name)
        except EOFError:
            break
            
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
