#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import yaml
import os

def main():
    rclpy.init()
    
    # BasicNavigator provides a simple Python API for interacting with Nav2
    navigator = BasicNavigator()
    
    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    
    waypoint_file = os.path.expanduser('~/waypoints.yaml')
    if not os.path.exists(waypoint_file):
        print(f"Error: {waypoint_file} does not exist.")
        rclpy.shutdown()
        return
        
    with open(waypoint_file, 'r') as f:
        waypoints = yaml.safe_load(f)
        
    if not waypoints:
        print("No waypoints found in file.")
        rclpy.shutdown()
        return
        
    while True:
        print("\n=== Available Waypoints ===")
        for wp_name in sorted(waypoints.keys()):
            print(f"  {wp_name}")
        print("  exit")
            
        choice = input("\nEnter waypoint to navigate to: ").strip()
        
        if choice.lower() == 'exit':
            break
            
        if choice not in waypoints:
            print(f"Invalid waypoint '{choice}'. Please type one of the names listed above.")
            continue
            
        wp = waypoints[choice]
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = float(wp['position']['x'])
        goal_pose.pose.position.y = float(wp['position']['y'])
        goal_pose.pose.position.z = float(wp['position']['z'])
        
        goal_pose.pose.orientation.x = float(wp['orientation']['x'])
        goal_pose.pose.orientation.y = float(wp['orientation']['y'])
        goal_pose.pose.orientation.z = float(wp['orientation']['z'])
        goal_pose.pose.orientation.w = float(wp ['orientation']['w'])
        
        print(f"\nNavigating to {choice}...")
        navigator.goToPose(goal_pose)
        
        # Loop until task completes
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 20 == 0:
                print(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
                
        # Check result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(">>> Goal succeeded! Reached destination.")
        elif result == TaskResult.CANCELED:
            print(">>> Goal was canceled!")
        elif result == TaskResult.FAILED:
            print(">>> Goal failed!")

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
