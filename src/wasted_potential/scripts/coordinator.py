#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import yaml
import os
import subprocess
import time

def main():
    rclpy.init()
    
    # 1. Start Navigator
    navigator = BasicNavigator()
    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    
    # 2. Tuck arms out of view
    print("Moving arms to safe observation pose...")
    subprocess.run(["ros2", "run", "wasted_potential", "prepare_perception.py"])
    time.sleep(2)
    
    # 3. Load Waypoints
    waypoint_file = os.path.expanduser('~/waypoints.yaml')
    if not os.path.exists(waypoint_file):
        print(f"Error: {waypoint_file} does not exist.")
        rclpy.shutdown()
        return
        
    with open(waypoint_file, 'r') as f:
        waypoints = yaml.safe_load(f)
        
    if not waypoints:
        print("No waypoints found.")
        rclpy.shutdown()
        return
        
    while True:
        # Prompt for Shelf
        print("\n=== Available Waypoints ===")
        for wp_name in sorted(waypoints.keys()):
            print(f"  {wp_name}")
        print("  exit")
            
        shelf_choice = input("\nEnter the shelf/waypoint to navigate to: ").strip()
        
        if shelf_choice.lower() == 'exit':
            break
            
        if shelf_choice not in waypoints:
            print(f"Invalid waypoint '{shelf_choice}'.")
            continue
            
        # Prompt for Book Color
        colors = ['red', 'green', 'blue', 'yellow']
        color_choice = input("Enter the target book color (red, green, blue, yellow): ").strip().lower()
        if color_choice not in colors:
            print("Invalid color.")
            continue
            
        # 4. Set the Perception Node Parameter
        print(f"Setting perception node to look for {color_choice}...")
        subprocess.run(["ros2", "param", "set", "/perception_node", "target_colour", color_choice])
        
        # 5. Navigate to Shelf
        wp = waypoints[shelf_choice]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = float(wp['position']['x'])
        goal_pose.pose.position.y = float(wp['position']['y'])
        goal_pose.pose.position.z = float(wp['position']['z'])
        goal_pose.pose.orientation.x = float(wp['orientation']['x'])
        goal_pose.pose.orientation.y = float(wp['orientation']['y'])
        goal_pose.pose.orientation.z = float(wp['orientation']['z'])
        goal_pose.pose.orientation.w = float(wp['orientation']['w'])
        
        print(f"\nNavigating to {shelf_choice}...")
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
            print(f">>> The robot is now scanning for the {color_choice} book!")
            
            import re
            match = re.search(r'\d+', shelf_choice)
            if match:
                shelf_num = match.group()
                cmd = ["ros2", "launch", "wasted_potential", "solution.launch.py", f"shelf_column_number:={shelf_num}", f"book_colour:={color_choice}"]
                print(">>> Launching Vision & Grasping Pipeline...")
                subprocess.run(cmd)
            else:
                print("Error: Could not extract column number from waypoint name.")
        elif result == TaskResult.CANCELED:
            print(">>> Goal was canceled!")
        elif result == TaskResult.FAILED:
            print(">>> Goal failed!")

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
