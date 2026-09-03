#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import time
import os
import ikpy.chain
import numpy as np
from scipy.spatial.transform import Rotation
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Bool

class GraspCoordinator(Node):
    def __init__(self):
        super().__init__('grasp_coordinator')
        
        # Load IK Chain
        urdf_path = "/opt/erc_ws/src/erc_description/urdf/tiago_pro.urdf"
        try:
            self.arm_chain = ikpy.chain.Chain.from_urdf_file(
                urdf_path,
                base_elements=[
                    "base_link",
                    "torso_lift_joint", "torso_lift_link",
                    "arm_right_1_joint", "arm_right_1_link",
                    "arm_right_2_joint", "arm_right_2_link",
                    "arm_right_3_joint", "arm_right_3_link",
                    "arm_right_4_joint", "arm_right_4_link",
                    "arm_right_5_joint", "arm_right_5_link",
                    "arm_right_6_joint", "arm_right_6_link",
                    "arm_right_7_joint", "arm_right_7_link",
                    "gripper_right_base_joint", "gripper_right_base_link",
                    "gripper_right_grasping_frame_joint", "gripper_right_grasping_link"
                ],
                active_links_mask=[False, False, True, True, True, True, True, True, True, False, False]
            )
            self.get_logger().info("IK Chain loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load IK chain: {e}")
            self.arm_chain = None
        
        self.right_arm_pub = self.create_publisher(JointTrajectory, '/arm_right_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)
        self.gripper_right_pub = self.create_publisher(JointTrajectory, '/gripper_right_controller/joint_trajectory', 10)
        self.gripper_left_pub = self.create_publisher(JointTrajectory, '/gripper_left_controller/joint_trajectory', 10)
        
        # TF2 Setup for transforming coordinates
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.pose_sub = self.create_subscription(PoseStamped, '/erc/book_3d_pose', self.pose_callback, 10)
        self.approach_sub = self.create_subscription(Bool, '/erc/approach_complete', self.approach_callback, 10)
        self.tilt_head_sub = self.create_subscription(Bool, '/erc/tilt_head', self.tilt_head_callback, 10)
        self.align_sub = self.create_subscription(Bool, '/erc/alignment_complete', self.align_callback, 10)
        self.proceed_sub = self.create_subscription(Bool, '/erc/proceed_grasp', self.proceed_callback, 10)
        self.proceed_extract_sub = self.create_subscription(Bool, '/erc/proceed_extract', self.proceed_extract_callback, 10)
        self.proceed_extract_flag = False
        self.arm_ready_pub = self.create_publisher(Bool, '/erc/arm_ready', 10)
        
        from geometry_msgs.msg import Twist
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.gripper_position_left = 0.069 # Default open
        self.gripper_position_right = 0.069
        
        from nav_msgs.msg import Odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_odom = None
        
        self.grasping = False
        self.approach_complete = False
        self.alignment_complete = False
        
        self.contact_detected = False
        try:
            from ros_gz_interfaces.msg import Contacts
            self.contact_sub = self.create_subscription(Contacts, '/contacts', self.contacts_callback, 10)
        except ImportError:
            self.get_logger().error("ros_gz_interfaces not found, contact sensing will not work!")
        
        # Head control for sweeping
        self.head_pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.sweep_timer = self.create_timer(0.1, self.sweep_callback)
        self.sweep_start_time = None
        
        self.get_logger().info("Grasp Coordinator ready. Waiting for visual alignment to complete...")

    def joint_states_callback(self, msg):
        try:
            if 'gripper_right_finger_joint' in msg.name:
                idx = msg.name.index('gripper_right_finger_joint')
                self.gripper_position_right = msg.position[idx]
            if 'gripper_left_finger_joint' in msg.name:
                idx = msg.name.index('gripper_left_finger_joint')
                self.gripper_position_left = msg.position[idx]
                
            if not hasattr(self, 'current_arm_positions'):
                self.current_arm_positions = [0.0] * 7
            for i in range(1, 8):
                joint_name = f'arm_right_{i}_joint'
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_arm_positions[i-1] = msg.position[idx]
                    
            if 'torso_lift_joint' in msg.name:
                idx = msg.name.index('torso_lift_joint')
                self.current_torso_position = msg.position[idx]
        except ValueError:
            pass

    def odom_callback(self, msg):
        self.current_odom = msg

    def contacts_callback(self, msg):
        for contact in msg.contacts:
            name1 = contact.collision1.name if hasattr(contact, 'collision1') else ""
            name2 = contact.collision2.name if hasattr(contact, 'collision2') else ""
            
            # Check if any part of the gripper is touching any book
            gripper_in_contact = 'gripper' in name1 or 'gripper' in name2
            book_in_contact = 'book' in name1 or 'book' in name2
            
            if gripper_in_contact and book_in_contact:
                self.contact_detected = True

    def approach_callback(self, msg):
        self.approach_complete = msg.data
        if self.approach_complete and not self.grasping:
            self.get_logger().info("Approach complete! Sending arm_ready signal immediately...")
            
            # We don't prepare the arm here anymore. We wait for alignment and use IK.
            # But we must tell the alignment node that we are ready!
            ready_msg = Bool()
            ready_msg.data = True
            self.arm_ready_pub.publish(ready_msg)
        else:
            self.get_logger().warn("Never saw the book from far away! Defaulting to sweep...")
            self.sweep_start_time = None

    def tilt_head_callback(self, msg):
        if not self.grasping and hasattr(self, 'far_z'):
            import math
            tilt = -math.atan2(1.40 - self.far_z, 0.8)
            self.get_logger().info(f"Book far Z is {self.far_z:.2f}m. Tilting head to {tilt:.2f} rad.")
            
            msg_head = JointTrajectory()
            msg_head.joint_names = ['head_1_joint', 'head_2_joint']
            point = JointTrajectoryPoint()
            point.positions = [0.0, tilt]
            point.time_from_start = Duration(sec=1, nanosec=0)
            msg_head.points.append(point)
            self.head_pub.publish(msg_head)

    def arm_ready_callback(self):
        self.head_moving = False
        msg = Bool()
        msg.data = True
        self.arm_ready_pub.publish(msg)

    def sweep_callback(self):
        if not self.approach_complete or self.grasping or getattr(self, 'head_moving', False):
            return
            
        if hasattr(self, 'far_z'): # Don't sweep if we successfully targeted
            return
            
        if self.sweep_start_time is None:
            self.sweep_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("Starting head sweep to search for the book...")
            
        t = (self.get_clock().now().nanoseconds / 1e9) - self.sweep_start_time
        import math
        
        # Sweep between 0.0 (straight) and -0.6 (down)
        tilt = -0.3 + 0.3 * math.cos(t * 1.0) # 1.0 rad/s sweeping speed
        
        msg = JointTrajectory()
        msg.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, tilt]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        msg.points.append(point)
        self.head_pub.publish(msg)

    def align_callback(self, msg):
        if msg.data and not getattr(self, 'alignment_complete', False):
            self.get_logger().info("Alignment complete! Waiting 2 seconds for camera and TF to settle...")
            # We don't sleep in the callback thread directly, instead we just set a timer
            # or record the time it was completed.
            self.alignment_time = self.get_clock().now().nanoseconds / 1e9
        self.alignment_complete = msg.data

    def pose_callback(self, msg):
        if self.grasping:
            return
            
        if getattr(self, 'alignment_complete', False):
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - getattr(self, 'alignment_time', current_time) < 2.0:
                # Ignore stale coordinates while camera settles
                return
            
        try:
            # Look up the transform from camera to base_link
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time()
            )
            
            # Transform the book pose to the base_link frame
            pose_base = tf2_geometry_msgs.do_transform_pose(msg.pose, trans)
            
            if not self.approach_complete or getattr(self, 'head_moving', False):
                self.far_z = pose_base.position.z
                return
            
            # If approach is complete but close horizontal alignment is NOT complete yet, we wait.
            # We ONLY grasp once alignment_complete is True.
            if not self.alignment_complete:
                return
            
            book_x = pose_base.position.x
            book_y = pose_base.position.y
            book_z = pose_base.position.z
            
            # Target the exact center of the book to prevent center-of-mass torque!
            self.target_xyz = [book_x, book_y, book_z]
            self.get_logger().info(f"Book transformed to base_link: X={book_x:.2f}, Y={book_y:.2f}, Z={book_z:.2f}")
            self.execute_grasp()
            
        except Exception as e:
            # We don't want to spam the logs or crash the node on transient TF errors
            pass

    def execute_grasp(self):
        self.grasping = True
        self.get_logger().info(f"\n\n=======================================================\nROBOT HORIZONTALLY ALIGNED! WAITING FOR AUTHORIZATION!\nRUN THIS COMMAND TO PROCEED:\nros2 topic pub --once /erc/proceed_grasp std_msgs/msg/Bool '{{data: true}}'\n=======================================================\n")
        
    def proceed_extract_callback(self, msg):
        self.proceed_extract_flag = msg.data

    def proceed_callback(self, msg):
        if not self.grasping or getattr(self, 'grasp_executed', False):
            return
        
        self.grasp_executed = True
        def run_grasp():
            def sim_sleep(duration):
                start = self.get_clock().now().nanoseconds / 1e9
                while (self.get_clock().now().nanoseconds / 1e9) - start < duration:
                    time.sleep(0.05)
            
            def drive_distance(dist):
                import math
                if not self.current_odom:
                    sim_sleep(dist / 0.1)
                    return
                start_x = self.current_odom.pose.pose.position.x
                start_y = self.current_odom.pose.pose.position.y
                start_time = self.get_clock().now().nanoseconds / 1e9
                while True:
                    self.cmd_vel_pub.publish(twist)
                    curr_x = self.current_odom.pose.pose.position.x
                    curr_y = self.current_odom.pose.pose.position.y
                    d = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)
                    if d >= abs(dist):
                        break
                    
                    # Prevent infinite loop if we crash into the shelf
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    expected_time = abs(dist) / 0.1
                    if current_time - start_time > expected_time + 2.0:
                        self.get_logger().warn(f"Drive timeout! We probably hit the shelf. (d={d:.2f}m)")
                        break
                        
                    time.sleep(0.05)
            
            # --- COMPUTE IK FOR "READY" POSE ---
            # We compute IK for a safe "ready" pose 40cm in front of the book.
            # This prevents arm overextension and keeps the arm safely away from the shelf during deployment.
            if hasattr(self, 'target_xyz') and self.arm_chain is not None:
                self.get_logger().info("Computing Inverse Kinematics for safe ready pose...")
                target_position = np.array(self.target_xyz)
                
                # We want the ready pose to be 0.40m safely in front of the book!
                target_position[0] -= 0.40
                # Desired orientation: Perfectly horizontal and pointing forward!
                # This ensures the fingers are exactly parallel to the book's sides, maximizing contact area and friction.
                target_orientation = np.array([
                    [ 1.0,  0.0,  0.0],
                    [ 0.0, -1.0,  0.0],
                    [ 0.0,  0.0, -1.0]
                ])
                
                # Choose the ideal torso height AND snap Z to the exact shelf height!
                # The user's perfect preset grabbed the book exactly 5.5mm above the shelf surface.
                # If we aim for the book's centroid (which vision gives us), the arm pitches up and hits the shelf above.
                book_z = target_position[2]
                if book_z > 1.3:
                    ideal_torso = 0.35 # Row 1
                    target_position[2] = 1.355
                elif book_z > 0.95:
                    ideal_torso = 0.35 # Row 2
                    target_position[2] = 1.005
                elif book_z > 0.6:
                    ideal_torso = 0.15 # Row 3
                    target_position[2] = 0.655
                else:
                    ideal_torso = 0.0  # Row 4 (Bottom)
                    target_position[2] = 0.305
                    
                # Dynamic initial guess for arm joints based on shelf height!
                # Desired orientation: Perfectly horizontal and pointing forward!
                # We use perfectly horizontal presets for each row to ensure IKPy doesn't get stuck in slanted local minima!
                if book_z > 1.2:
                    # Row 4 (Top Shelf) Z ~ 1.33
                    initial_guess = [0.0, 0.35, 0.1248, 1.0585, -0.3825, -1.2932, 0.6047, 1.6895, 0.3412, 0.0, 0.0]
                elif book_z > 0.85:
                    # Row 3 Z ~ 1.0
                    initial_guess = [0.0, 0.3493, -0.3902, 0.7983, 0.3096, -1.8384, 0.0889, 2.0868, 0.0839, 0.0, 0.0]
                elif book_z > 0.5:
                    # Row 2 Z ~ 0.6
                    initial_guess = [0.0, 0.2133, -0.5443, 0.3318, 0.3880, -2.3748, 0.1885, 2.1447, 0.1961, 0.0, 0.0]
                else:
                    # Row 1 (Bottom Shelf) Z ~ 0.25
                    initial_guess = [0.0, 0.0, -0.1091, -1.6044, -0.3713, -2.4434, -0.0643, 0.3343, -0.0308, 0.0, 0.0]
                
                initial_guess[1] = ideal_torso
                
                best_pos_error = float('inf')
                best_ori_error = float('inf')
                best_joints = None
                
                import random
                for attempt in range(100):
                    # Add slight random noise to the initial guess to escape local minima
                    noisy_guess = [j + random.uniform(-0.1, 0.1) for j in initial_guess]
                    noisy_guess[0] = 0.0 # base_link fixed
                    noisy_guess[1] = ideal_torso # torso fixed
                    noisy_guess[-2:] = [0.0, 0.0] # fixed joints
                    
                    result_joints = self.arm_chain.inverse_kinematics(
                        target_position=target_position,
                        target_orientation=target_orientation,
                        orientation_mode="all",
                        initial_position=noisy_guess
                    )
                    
                    res_matrix = self.arm_chain.forward_kinematics(result_joints)
                    pos_err = np.linalg.norm(res_matrix[:3, 3] - target_position)
                    ori_err = np.linalg.norm(res_matrix[:3, :3] - target_orientation)
                    
                    if pos_err < best_pos_error:
                        best_pos_error = pos_err
                        best_ori_error = ori_err
                        best_joints = result_joints
                        
                    if pos_err < 0.02 and ori_err < 0.005:
                        self.get_logger().info(f"IK Perfect Solution Found! Attempt {attempt}. PosErr: {pos_err:.4f}, OriErr: {ori_err:.4f}")
                        break
                
                if best_pos_error > 0.05 or best_ori_error > 0.02:
                    self.get_logger().error(f"IK solver failed! Best PosErr: {best_pos_error:.3f}m, OriErr: {best_ori_error:.3f}. ABORTING GRASP.")
                    return
                else:
                    self.get_logger().info(f"IK Solved! Best PosErr: {best_pos_error:.4f}m, OriErr: {best_ori_error:.4f}")
                
                ik_torso = best_joints[1]
                ik_arm = best_joints[2:9].tolist()
                
                self.get_logger().info(f"Moving to Ready Pose... Torso: {ik_torso:.3f}")
                self.move_torso([ik_torso])
                self.move_arm(ik_arm)
                self.get_logger().info("Deploying arm and opening gripper in safe space...")
                self.move_gripper([0.069]) # Open gripper beforehand!
                sim_sleep(4.0) # Wait for arm to reach ready pose safely in the air
            else:
                self.get_logger().warn("No target_xyz or IK chain available! Aborting...")
                return
                
            # --- DRIVE FORWARD TO SLIDE HAND INTO BOOK ---
            # We need to cover the 0.40m gap, PLUS 0.08m deep grasp = 0.48m total drive
            drive_dist = 0.48
            self.get_logger().info(f"Driving {drive_dist}m forward to slide hand into book...")
            from geometry_msgs.msg import Twist
            twist = Twist()
            twist.linear.x = 0.1 # 0.1 m/s
            
            drive_distance(drive_dist)
            
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            sim_sleep(0.5)
            
            total_drive_dist = drive_dist
            
            # Robust Grasping Loop
            max_retries = 8
            for attempt in range(max_retries):
                self.get_logger().info(f"Grabbing... (Attempt {attempt+1})")
                self.contact_detected = False
                
                # We must carefully command the position to exactly 0.023m (a 4.6cm gap).
                # Since the book is 5cm wide, this generates exactly 2mm of penetration per side,
                # creating stable friction without violently repelling the book!
                self.move_gripper([0.023])
                sim_sleep(2.0) # Wait for gripper to close fully
                
                # Because the gripper's minimum gap is exactly 3cm (the width of the book),
                # the joint position will reach 0.000 whether it's empty or grabbing the book!
                # Therefore, we MUST rely on the Gazebo contact sensor!
                if self.contact_detected:
                    self.get_logger().info("SUCCESS! Contact sensor triggered, book acquired!")
                    
                    self.get_logger().info(f"\n\n=======================================================\nBOOK GRIPPED! WAITING FOR AUTHORIZATION TO EXTRACT!\nRUN THIS COMMAND TO PROCEED:\nros2 topic pub --once /erc/proceed_extract std_msgs/msg/Bool '{{data: true}}'\n=======================================================\n")
                    while not self.proceed_extract_flag:
                        sim_sleep(0.5)
                        
                    # Phase 1: Straight Extraction
                    self.get_logger().info("Grasp confirmed. Phase 1: Extracting straight back...")
                    twist.linear.x = -0.1
                    drive_distance(0.20)
                    
                    # Phase 2: Curving Extraction
                    self.get_logger().info("Phase 2: Pre-calculating curving extraction trajectory...")
                    
                    extraction_dist = 0.28
                    extraction_duration = extraction_dist / 0.1
                    steps = 30
                    dt = extraction_duration / steps
                    
                    trajectory_joints = []
                    current_guess = list(best_joints)
                    
                    import math
                    import numpy as np
                    for step in range(steps):
                        progress = step / float(steps - 1)
                        # Pitch up to 75 degrees
                        pitch_angle = progress * math.radians(75) 
                        # Lift up to 10cm to ensure the wrist clears the shelf edge as it pitches
                        current_z = self.target_xyz[2] + progress * 0.10
                        
                        current_target = [self.target_xyz[0], self.target_xyz[1], current_z]
                        R_pitch = np.array([
                            [math.cos(pitch_angle), 0, math.sin(pitch_angle)],
                            [0, 1, 0],
                            [-math.sin(pitch_angle), 0, math.cos(pitch_angle)]
                        ])
                        current_orientation = target_orientation.dot(R_pitch)
                        
                        res = self.arm_chain.inverse_kinematics(
                            target_position=current_target,
                            target_orientation=current_orientation,
                            orientation_mode="all",
                            initial_position=current_guess
                        )
                        current_guess = list(res)
                        trajectory_joints.append(res[2:9].tolist())
                        
                    self.get_logger().info("Executing curve extraction...")
                    twist.linear.x = -0.1
                    for step in range(steps):
                        self.cmd_vel_pub.publish(twist)
                        self.move_arm(trajectory_joints[step])
                        sim_sleep(dt)
                        
                    # Stop base
                    twist.linear.x = 0.0
                    self.cmd_vel_pub.publish(twist)
                    self.cmd_vel_pub.publish(twist)
                    self.get_logger().info("Extraction complete! The book should be balancing on the hand!")
                    
                    break
                else:
                    self.get_logger().warn("MISSED! No contact detected. Retrying...")
                    self.move_gripper([0.069]) # Open gripper
                    sim_sleep(1.5) # Wait for gripper to open fully
                    
                    # DO NOT drive forward here, because the arm is already extended!
                    # Driving forward would crash the arm into the shelf!
                    
                    total_drive_dist += 0.0
            
            # (Extraction handled internally by the curved trajectory block)
            
            # Safe stow pose
            self.move_arm([-0.36, -1.83, -0.47, -2.35, -0.0, -1.2, 0.0])
            sim_sleep(4.0)
            
            self.get_logger().info("GRASP COMPLETE!")
            
        import threading
        threading.Thread(target=run_grasp, daemon=True).start()
        
    def move_arm(self, positions):
        msg = JointTrajectory()
        msg.joint_names = [f'arm_right_{i}_joint' for i in range(1, 8)]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=3, nanosec=0)
        msg.points.append(point)
        self.right_arm_pub.publish(msg)
        
    def move_torso(self, positions):
        msg = JointTrajectory()
        msg.joint_names = ['torso_lift_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=3, nanosec=0)
        msg.points.append(point)
        self.torso_pub.publish(msg)
        
    def move_gripper(self, positions):
        val = positions[0]
        
        msg_right = JointTrajectory()
        msg_right.joint_names = ['gripper_right_finger_joint']
        point_right = JointTrajectoryPoint()
        point_right.positions = [val]
        point_right.time_from_start = Duration(sec=1, nanosec=0)
        msg_right.points.append(point_right)
        self.gripper_right_pub.publish(msg_right)
        
        msg_left = JointTrajectory()
        msg_left.joint_names = ['gripper_left_finger_joint']
        point_left = JointTrajectoryPoint()
        point_left.positions = [val]
        point_left.time_from_start = Duration(sec=1, nanosec=0)
        msg_left.points.append(point_left)
        self.gripper_left_pub.publish(msg_left)


def main(args=None):
    rclpy.init(args=args)
    node = GraspCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
