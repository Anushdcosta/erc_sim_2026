import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class SaveClean(Node):
    def __init__(self):
        super().__init__('save_clean')
        self.sub = self.create_subscription(Image, '/head_front_camera/head_front_camera/color/image_raw', self.cb, 1)
        self.bridge = CvBridge()
        self.saved = False
        
    def cb(self, msg):
        if not self.saved:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite('/root/erc_images/clean.jpg', cv_image)
            print("Saved clean.jpg")
            self.saved = True
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SaveClean()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
