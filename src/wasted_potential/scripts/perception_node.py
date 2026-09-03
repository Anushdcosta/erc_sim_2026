#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        self.declare_parameter('target_colour', 'red')
        self.target_colour = self.get_parameter('target_colour').value.lower()
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, '/head_front_camera/head_front_camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/head_front_camera/head_front_camera/depth/image_rect_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/head_front_camera/head_front_camera/depth/camera_info', self.info_callback, 10)
            
        self.annotated_pub = self.create_publisher(Image, '/erc/annotated_image', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/erc/book_3d_pose', 10)
        
        self.image_save_dir = os.path.expanduser('~/erc_images')
        os.makedirs(self.image_save_dir, exist_ok=True)
        
        self.get_logger().info(f"Perception node started! Looking for {self.target_colour} book.")
        
        self.last_save_time = self.get_clock().now()
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        from std_msgs.msg import Bool
        self.snapshot_sub = self.create_subscription(Bool, '/erc/take_snapshot', self.snapshot_callback, 10)
        self.take_snapshot = False
        
        self.depth_image = None
        self.camera_info = None
        
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.locked = False
        self.target_book_odom_pose = None
        self.get_logger().info("Perception Node initialized.")
        
    def snapshot_callback(self, msg):
        if msg.data and hasattr(self, 'last_image') and not self.locked:
            try:
                import cv2
                cv2.imwrite('/root/full_shelf_map.jpg', self.last_image)
                self.get_logger().info("Captured FULL SHELF map at /root/full_shelf_map.jpg!")
                
                # Lock the target book's position in the global odom frame
                if hasattr(self, 'target_book_pose_camera_frame'):
                    import rclpy
                    import tf2_geometry_msgs
                    from geometry_msgs.msg import PoseStamped
                    
                    try:
                        t = self.tf_buffer.lookup_transform('odom', self.camera_info.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
                        
                        # Create PoseStamped in camera frame
                        cam_pose = PoseStamped()
                        cam_pose.header.frame_id = self.camera_info.header.frame_id
                        cam_pose.header.stamp = self.get_clock().now().to_msg()
                        cam_pose.pose.position.x = self.target_book_pose_camera_frame[0]
                        cam_pose.pose.position.y = self.target_book_pose_camera_frame[1]
                        cam_pose.pose.position.z = self.target_book_pose_camera_frame[2]
                        cam_pose.pose.orientation.w = 1.0
                        
                        # Transform to odom
                        odom_pose = tf2_geometry_msgs.do_transform_pose(cam_pose.pose, t)
                        
                        self.target_book_odom_pose = (odom_pose.position.x, odom_pose.position.y, odom_pose.position.z)
                        self.locked = True
                        self.get_logger().info(f"FROZE target book pose in odom frame: {self.target_book_odom_pose}")
                    except Exception as e:
                        self.get_logger().error(f"TF lookup failed: {e}")
                else:
                    self.get_logger().warn("Could not freeze target book pose - book not found yet!")
            except Exception as e:
                self.get_logger().error(f"Failed to freeze perception: {e}")
        
    def depth_callback(self, msg):
        try:
            # Depth images can be 32FC1 (meters) or 16UC1 (millimeters)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth Bridge error: {e}")
            
    def info_callback(self, msg):
        self.camera_info = msg
        
    def parameters_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        success = True
        for param in params:
            if param.name == 'target_colour':
                if param.type_ == param.Type.STRING:
                    self.target_colour = param.value.lower()
                    self.get_logger().info(f"Target colour updated dynamically to: {self.target_colour}")
        return SetParametersResult(successful=success)

    def image_callback(self, msg):
        import cv2
        from cv_bridge import CvBridge
        import numpy as np
        bridge = CvBridge()
        
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_image = cv_image
        
        # If locked, bypass all OpenCV processing and just publish the static locked pose
        if self.locked and self.target_book_odom_pose is not None:
            self.annotated_pub.publish(bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
            
            # Continuously publish the static odom pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'odom'
            pose_msg.pose.position.x = self.target_book_odom_pose[0]
            pose_msg.pose.position.y = self.target_book_odom_pose[1]
            pose_msg.pose.position.z = self.target_book_odom_pose[2]
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)
            return

        if self.depth_image is None or self.camera_info is None:
            return
            
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        bounds = {
            'red': [(0, 100, 100), (10, 255, 255)], 
            'green': [(40, 50, 50), (80, 255, 255)],
            'blue': [(100, 100, 50), (130, 255, 255)],
            'yellow': [(20, 100, 100), (40, 255, 255)]
        }
        
        try:
            import rclpy
            t_cam = self.tf_buffer.lookup_transform('base_link', self.camera_info.header.frame_id, rclpy.time.Time())
            
            tz = t_cam.transform.translation.z
            qx = t_cam.transform.rotation.x
            qy = t_cam.transform.rotation.y
            qz = t_cam.transform.rotation.z
            qw = t_cam.transform.rotation.w
            
            # 3rd row of rotation matrix to extract Z
            r20 = 2 * (qx*qz - qy*qw)
            r21 = 2 * (qy*qz + qx*qw)
            r22 = 1 - 2 * (qx**2 + qy**2)
        except Exception as e:
            return
            
        all_books = []
        
        # 1. Detect all books of all colors
        for color_name, (lower, upper) in bounds.items():
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            if color_name == 'red':
                mask2 = cv2.inRange(hsv_image, np.array([170, 100, 100]), np.array([180, 255, 255]))
                mask = cv2.bitwise_or(mask, mask2)
                
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours: continue
            
            valid_contours = [c for c in contours if cv2.contourArea(c) > 20]
            for c in valid_contours:
                x, y, w, h = cv2.boundingRect(c)
                cx = int(x + w/2)
                cy = int(y + h/2)
                
                # Make sure we don't go out of bounds
                if cy < self.depth_image.shape[0] and cx < self.depth_image.shape[1]:
                    depth = self.depth_image[cy, cx]
                    
                    if self.depth_image.dtype == np.uint16:
                        z = depth / 1000.0
                    else:
                        z = float(depth)
                        
                    if z <= 0.1: continue # Invalid depth
                    
                    # Calculate 3D coordinates in optical frame
                    fx = self.camera_info.k[0]
                    cx_cam = self.camera_info.k[2]
                    fy = self.camera_info.k[4]
                    cy_cam = self.camera_info.k[5]
                    
                    x_3d = (cx - cx_cam) * z / fx
                    y_3d = (cy - cy_cam) * z / fy
                    
                    # Transform to base_link to get the absolute height from the floor
                    abs_z = (r20 * x_3d) + (r21 * y_3d) + (r22 * z) + tz
                    
                    if abs_z > 1.3:
                        row_num = 1
                    elif abs_z > 0.95:
                        row_num = 2
                    elif abs_z > 0.6:
                        row_num = 3
                    else:
                        row_num = 4
                        
                    all_books.append({
                        'color': color_name,
                        'x': x, 'y': y, 'w': w, 'h': h,
                        'cx': cx, 'cy': cy, 'z': z,
                        'row': row_num
                    })
                    
        # 2. Cluster books into Columns
        if all_books:
            all_books.sort(key=lambda b: b['cx'])
            columns = []
            current_col = [all_books[0]]
            for b in all_books[1:]:
                # If horizontal pixel gap is > 40, it's a new shelf column
                if b['cx'] - current_col[-1]['cx'] > 40:
                    columns.append(current_col)
                    current_col = [b]
                else:
                    current_col.append(b)
            columns.append(current_col)
            
            # Assign column numbers
            for col_idx, col_books in enumerate(columns):
                for b in col_books:
                    b['col'] = col_idx + 1

        # 3. Annotate Image and Find Target Book
        target_book = None
        min_dist = float('inf')
        img_center_x = cv_image.shape[1] / 2
        
        for b in all_books:
            # Draw bounding box
            cv2.rectangle(cv_image, (b['x'], b['y']), (b['x']+b['w'], b['y']+b['h']), (0, 255, 0), 2)
            
            # Draw annotation text
            text = f"C{b.get('col', '?')} R{b['row']}: {b['color'].upper()}"
            cv2.putText(cv_image, text, (b['x'], b['y']-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
            # Target identification for the grasping system
            if b['color'] == self.target_colour:
                dist = abs(b['cx'] - img_center_x)
                # Simply take the target-color book closest to the center
                if dist < min_dist:
                    min_dist = dist
                    target_book = b
                    
        # 4. Publish Target Pose
        if target_book:
            b = target_book
            fx = self.camera_info.k[0]
            cx_cam = self.camera_info.k[2]
            fy = self.camera_info.k[4]
            cy_cam = self.camera_info.k[5]
            
            x_3d = (b['cx'] - cx_cam) * b['z'] / fx
            y_3d = (b['cy'] - cy_cam) * b['z'] / fy
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.camera_info.header.frame_id
            pose.pose.position.x = x_3d
            pose.pose.position.y = y_3d
            pose.pose.position.z = b['z']
            pose.pose.orientation.w = 1.0
            
            self.pose_pub.publish(pose)
            
            # Save the raw camera optical frame coords temporarily so we can lock them on snapshot
            self.target_book_pose_camera_frame = (x_3d, y_3d, b['z'])
            
        # 5. Publish and Save Map
        # Only publish the annotated image if NOT locked (we already returned early if locked)
        self.annotated_pub.publish(bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
