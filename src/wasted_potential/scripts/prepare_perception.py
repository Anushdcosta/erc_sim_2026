#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class PreparePerceptionNode(Node):
    def __init__(self):
        super().__init__('prepare_perception')
        
        self.left_arm_pub = self.create_publisher(JointTrajectory, '/arm_left_controller/joint_trajectory', 10)
        self.right_arm_pub = self.create_publisher(JointTrajectory, '/arm_right_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)
        
        # Give publishers time to connect
        time.sleep(1.0)
        self.tuck_arms()
        
    def tuck_arms(self):
        self.get_logger().info("Tucking arms out of camera view...")
        
        # Left arm
        left_msg = JointTrajectory()
        left_msg.joint_names = [f'arm_left_{i}_joint' for i in range(1, 8)]
        
        left_point = JointTrajectoryPoint()
        # Official TIAGo Pro 'home' pose for left arm (T-Rex pose)
        left_point.positions = [0.36, -1.83, 0.47, -2.35, 0.0, -1.2, 0.0]
        left_point.time_from_start = Duration(sec=3, nanosec=0)
        left_msg.points.append(left_point)
        
        # Right arm
        right_msg = JointTrajectory()
        right_msg.joint_names = [f'arm_right_{i}_joint' for i in range(1, 8)]
        
        right_point = JointTrajectoryPoint()
        # Official TIAGo Pro 'home' pose for right arm (T-Rex pose)
        right_point.positions = [-0.36, -1.83, -0.47, -2.35, -0.0, -1.2, 0.0]
        right_point.time_from_start = Duration(sec=3, nanosec=0)
        right_msg.points.append(right_point)
        
        # Torso
        torso_msg = JointTrajectory()
        torso_msg.joint_names = ['torso_lift_joint']
        torso_point = JointTrajectoryPoint()
        torso_point.positions = [0.17]
        torso_point.time_from_start = Duration(sec=3, nanosec=0)
        torso_msg.points.append(torso_point)
        
        self.left_arm_pub.publish(left_msg)
        self.right_arm_pub.publish(right_msg)
        self.torso_pub.publish(torso_msg)
        
        self.get_logger().info("Arm commands sent. Waiting 3 seconds for movement to complete...")
        time.sleep(3.5)
        self.get_logger().info("Ready for perception!")

def main(args=None):
    rclpy.init(args=args)
    node = PreparePerceptionNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
