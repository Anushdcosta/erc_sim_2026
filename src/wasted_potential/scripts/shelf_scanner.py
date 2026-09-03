#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pytesseract
from pytesseract import Output
import time

class ShelfScanner(Node):
    def __init__(self):
        super().__init__('shelf_scanner')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/head_front_camera/head_front_camera/color/image_raw',
            self.image_callback,
            10)
        self.get_logger().info("Shelf Scanner started. Waiting for image...")
        self.shelf_mapping = {}
        self.scan_complete = False

    def image_callback(self, msg):
        if self.scan_complete:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
            
        # Preprocessing for OCR
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Apply a binary threshold (Adaptive thresholding is usually better for Gazebo shadows)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 10)
        
        found_items = {} # Maps digit -> x_coordinate
        
        # We run Tesseract twice: once for clustered text (PSM 6) and once for sparse text (PSM 11)
        for psm in [6, 11]:
            custom_config = f'--oem 3 --psm {psm} -c tessedit_char_whitelist=12345'
            d = pytesseract.image_to_data(thresh, output_type=Output.DICT, config=custom_config)
            
            n_boxes = len(d['text'])
            for i in range(n_boxes):
                # Clean text to just digits
                text = ''.join(filter(str.isdigit, d['text'][i]))
                if text in ['1', '2', '3', '4', '5']:
                    # Only add if we haven't seen this digit yet
                    if int(text) not in found_items:
                        found_items[int(text)] = d['left'][i]
        
        digits_found = list(found_items.keys())
        self.get_logger().info(f"Scanning... Numbers found so far: {digits_found}")
        
        # Check if we found exactly the 5 digits we need
        if len(digits_found) == 5:
            self.get_logger().info(f"Successfully read all 5 numbers!")
            
            # Sort the items by their X coordinate to determine physical columns (left-to-right)
            sorted_items = sorted(found_items.items(), key=lambda x: x[1])
            
            for index, (shelf_number, x_coord) in enumerate(sorted_items):
                physical_column = index + 1 # 1 to 5
                self.shelf_mapping[shelf_number] = f"shelf{physical_column}"
                
            self.get_logger().info(f"Final Mapping: {self.shelf_mapping}")
            self.scan_complete = True
            cv2.imwrite('/root/erc_images/ocr_scan.jpg', cv_image)

def main(args=None):
    rclpy.init(args=args)
    node = ShelfScanner()
    
    # Wait until scan is complete
    while rclpy.ok() and not node.scan_complete:
        rclpy.spin_once(node)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
