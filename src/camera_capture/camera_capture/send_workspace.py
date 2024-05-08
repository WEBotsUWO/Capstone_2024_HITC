#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import workspace_processing as wp


class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(Image, 'image_raw', self.send_image_callback, 10)
    

    def send_image_callback(self, data):
        wp.get_maze_details()

        
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber("image_processing")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
