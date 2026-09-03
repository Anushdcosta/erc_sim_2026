#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pytesseract
import sys

class VisualCoordinator(Node):
    def __init__(self, target_shelf):
        super().__init__('visual_coordinator')
        self.target_shelf = int(target_shelf)
        
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool
        self.bridge = CvBridge()
        
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/head_front_camera/head_front_camera/color/image_raw', self.image_callback, 1)
        self.depth_sub = self.create_subscription(Image, '/head_front_camera/head_front_camera/depth/image_rect_raw', self.depth_callback, 1)
        self.pose_sub = self.create_subscription(PoseStamped, '/erc/book_3d_pose', self.pose_callback, 10)
        
        self.approach_pub = self.create_publisher(Bool, '/erc/approach_complete', 10)
        self.align_pub = self.create_publisher(Bool, '/erc/alignment_complete', 10)
        self.snapshot_pub = self.create_publisher(Bool, '/erc/take_snapshot', 10)
        self.tilt_head_pub = self.create_publisher(Bool, '/erc/tilt_head', 10)
        
        self.arm_ready_sub = self.create_subscription(Bool, '/erc/arm_ready', self.arm_ready_callback, 10)
        self.arm_ready = False
        
        self.bridge = CvBridge()
        
        self.state = 'SCANNING'
        self.current_error_x = None
        self.search_direction = -100
        self.current_depth = None
        self.close_error_x = None
        
        # P-Controller constants
        self.kp_y = 0.002
        self.tolerance_x = 5 # pixels
        self.target_center_x = 320
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Visual Coordinator started. Target shelf: {self.target_shelf}")
        self.get_logger().info("Rotating to search for all 5 shelves...")
        
    def arm_ready_callback(self, msg):
        self.arm_ready = msg.data

    def timer_callback(self):
        twist = Twist()
        if self.state == 'SCANNING':
            # Spin slowly to the right
            twist.angular.z = -0.2
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == 'STRAFING':
            if self.current_error_x is not None:
                if abs(self.current_error_x) < self.tolerance_x:
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().info(f"ALIGNED! Target {self.target_shelf} is perfectly centered.")
                    self.state = 'DONE'
                else:
                    y_vel = self.kp_y * self.current_error_x
                    y_vel = max(min(y_vel, 0.2), -0.2)
                    twist.linear.y = y_vel
                    self.cmd_vel_pub.publish(twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                
        elif self.state == 'APPROACHING':
            if self.current_depth is not None:
                if self.current_depth <= 1.5:
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().info("Arrived at 1.5m safe distance! Tilting head down to see book...")
                    
                    from std_msgs.msg import Bool
                    msg = Bool()
                    msg.data = True
                    self.tilt_head_pub.publish(msg)
                    
                    self.state = 'PREPARE_ARM'
                else:
                    vel_x = 0.5 * (self.current_depth - 1.5)
                    vel_x = max(min(vel_x, 0.2), 0.05)
                    twist.linear.x = vel_x
                    self.cmd_vel_pub.publish(twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                
        elif self.state == 'PREPARE_ARM':
            self.cmd_vel_pub.publish(Twist())
            if not hasattr(self, 'prepare_start_time'):
                self.prepare_start_time = self.get_clock().now().nanoseconds / 1e9
                
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.prepare_start_time
            if self.arm_ready and elapsed > 3.0:
                self.get_logger().info("Arm is in spear position and head is tilted! Starting final approach...")
                self.state = 'FINAL_APPROACH'
                
        elif self.state == 'FINAL_APPROACH':
            if hasattr(self, 'book_x_base') and self.book_x_base is not None:
                # The robot wants to be exactly 1.1m away from the book itself
                if self.book_x_base <= 1.1:
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().info("Arrived exactly 1.1m from the book! Starting dynamic TF alignment...")
                    
                    # FREEZE the book's position in odom frame now that we are up close! (Zero odometry drift)
                    from std_msgs.msg import Bool
                    msg = Bool()
                    msg.data = True
                    self.snapshot_pub.publish(msg)
                    
                    self.state = 'CLOSE_ALIGNMENT'
                else:
                    vel_x = 0.5 * (self.book_x_base - 1.1)
                    vel_x = max(min(vel_x, 0.15), 0.05)
                    twist.linear.x = vel_x
                    self.cmd_vel_pub.publish(twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                
        elif self.state == 'CLOSE_ALIGNMENT':
            if self.close_error_x is not None:
                if abs(self.close_error_x) < 0.005: # 0.5 cm tolerance (hyper-precision)
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().info("Perfectly aligned horizontally with the GRIPPER! Triggering grasp!")
                    
                    from std_msgs.msg import Bool
                    msg = Bool()
                    msg.data = True
                    self.align_pub.publish(msg)
                    
                    self.state = 'DONE'
                else:
                    vel_y = self.close_error_x * 1.5
                    vel_y = max(min(vel_y, 0.1), -0.1) # Cap speed
                    twist.linear.y = vel_y
                    self.cmd_vel_pub.publish(twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                
        elif self.state == 'DONE':
            # Do NOT publish Twist here! grasp_coordinator needs to drive the robot.
            pass
            # self.get_logger().info("Robot is stopped. Waiting for grasp to complete.")

    def depth_callback(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            import numpy as np
            
            # Read depth at the center pixel
            h, w = depth_img.shape
            depth = depth_img[int(h/2), int(w/2)]
            
            if depth_img.dtype == np.uint16:
                depth_m = depth / 1000.0
            else:
                depth_m = float(depth)
                
            if depth_m > 0:
                self.current_depth = depth_m
                
        except Exception as e:
            pass

    def pose_callback(self, msg):
        if self.state in ['FINAL_APPROACH', 'CLOSE_ALIGNMENT']:
            try:
                import rclpy
                import tf2_geometry_msgs
                from geometry_msgs.msg import PoseStamped
                
                # 1. Transform book 3D coordinate to base_link
                try:
                    t_cam = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
                    
                    # Create PoseStamped in incoming frame (odom)
                    odom_pose = PoseStamped()
                    odom_pose.header = msg.header
                    odom_pose.pose = msg.pose
                    
                    # Transform to base_link
                    base_pose = tf2_geometry_msgs.do_transform_pose(odom_pose.pose, t_cam)
                    
                    self.book_x_base = base_pose.position.x
                    book_y_base = base_pose.position.y - 0.012 # Shifted right to prevent right finger from hitting spine
                except Exception as e:
                    self.get_logger().error(f"TF lookup failed: {e}")
                    return
                
                # 2. Look up gripper's Y coordinate in base_link
                t_grip = self.tf_buffer.lookup_transform('base_link', 'arm_right_tool_link', rclpy.time.Time())
                gripper_y_base = t_grip.transform.translation.y
                
                # 3. Calculate exact error
                self.close_error_x = book_y_base - gripper_y_base
            except Exception as e:
                self.get_logger().warn(f"TF Error in alignment: {e}")

    def image_callback(self, msg):
        if self.state not in ['SCANNING', 'STRAFING']:
            return
            
        try:
            full_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = full_cv_image.copy()
        except Exception as e:
            return
            
        h_full, w_full, _ = full_cv_image.shape
        crop_h = int(h_full / 3.0)
        cv_image = cv_image[0:crop_h, :]
            
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Use a strict threshold (numbers are pure black, background is white)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        
        found_items = {}
        
        # Robust OpenCV Contour Isolation
        # Instead of trusting Tesseract to find the characters, we physically isolate the black shapes ourselves
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            x, y, w_cnt, h_cnt = cv2.boundingRect(cnt)
            
            # The shelf numbers are roughly 40x40 pixels. Filter out tiny noise and giant blobs.
            if w_cnt > 10 and h_cnt > 15 and w_cnt < 80 and h_cnt < 80:
                roi = thresh[y:y+h_cnt, x:x+w_cnt]
                
                # Tesseract works best with black text on a white background with padding
                roi_inv = cv2.bitwise_not(roi)
                padded = cv2.copyMakeBorder(roi_inv, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=[255, 255, 255])
                
                # PSM 10 tells Tesseract to treat the image as a single, isolated character
                custom_config = '--oem 3 --psm 10 -c tessedit_char_whitelist=12345'
                char = pytesseract.image_to_string(padded, config=custom_config).strip()
                
                if char in ['1', '2', '3', '4', '5']:
                    x_center = x + w_cnt / 2.0
                    
                    if int(char) not in found_items:
                        found_items[int(char)] = x_center
                        cv2.rectangle(cv_image, (x, y), (x+w_cnt, y+h_cnt), (0, 255, 0), 2)
                        cv2.putText(cv_image, char, (x, max(0, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                        
        cv2.imwrite('/root/erc_images/ocr_annotated.jpg', cv_image)
        digits_found = list(found_items.keys())
        
        if self.state in ['SCANNING', 'STRAFING']:
            self.get_logger().info(f"Vision currently sees numbers: {digits_found}")
        
        if self.state == 'SCANNING':
            # REQUIRE all 5 shelves to be visible
            if len(digits_found) == 5:
                # To guarantee perfect perpendicularity (no angle skew), 
                # we don't stop immediately. We keep rotating until the 
                # center of the entire shelf structure is exactly at X=320.
                avg_x = sum(found_items.values()) / 5.0
                rot_error = self.target_center_x - avg_x
                
                if abs(rot_error) < 30: # 30 pixels tolerance for rotation
                    self.get_logger().info(f"Perpendicular! Shelf structure is perfectly centered. Transitioning to Strafing Mode.")
                    cv2.imwrite('/root/erc_images/ocr_all_shelves.jpg', cv_image)
                    
                    self.cmd_vel_pub.publish(Twist()) # Stop rotating immediately
                    self.state = 'STRAFING'
                
        if self.state == 'STRAFING':
            target_x = found_items.get(self.target_shelf)
            if target_x is not None:
                self.current_error_x = self.target_center_x - target_x
                self.get_logger().info(f"Target {self.target_shelf} found at X={target_x:.1f} (Error: {self.current_error_x:.1f})")
                self.search_direction = -100 # Reset to strafe right next time we lose it
                
                if abs(self.current_error_x) < self.tolerance_x:
                    # Create the spotlight effect on the FULL image
                    spotlight_img = full_cv_image.copy()
                    
                    # Draw a solid black rectangle on the left side
                    cv2.rectangle(spotlight_img, (0, 0), (self.target_center_x - 75, h_full), (0, 0, 0), -1)
                    # Draw a solid black rectangle on the right side
                    cv2.rectangle(spotlight_img, (self.target_center_x + 75, 0), (w_full, h_full), (0, 0, 0), -1)
                    
                    # Put a text overlay showing it's centered
                    cv2.putText(spotlight_img, f"TARGET {self.target_shelf} CENTERED", (self.target_center_x - 90, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    cv2.imwrite('/root/erc_images/ocr_aligned.jpg', spotlight_img)
                    
                    self.get_logger().info("Target centered horizontally! Transitioning to APPROACHING Mode.")
                    self.cmd_vel_pub.publish(Twist())
                    
                    # Trigger early arm preparation
                    from std_msgs.msg import Bool
                    msg = Bool()
                    msg.data = True
                    self.approach_pub.publish(msg)
                    self.get_logger().info("Triggered early arm preparation!")
                    
                    self.state = 'APPROACHING'
            else:
                self.get_logger().warn(f"Target {self.target_shelf} not visible! Strafing to search...")
                if len(digits_found) == 0:
                    self.search_direction *= -1
                    
                self.current_error_x = self.search_direction

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run wasted_potential visual_coordinator.py <target_shelf_number>")
        return
        
    target_shelf = sys.argv[1]
    
    node = VisualCoordinator(target_shelf)
    
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("visual_coordinator").info("Task Completed Successfully.")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
