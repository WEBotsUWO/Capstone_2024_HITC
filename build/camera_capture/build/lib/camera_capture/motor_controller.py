#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Bool
class CheckForSlipNode(Node):
    def __init__(self):
        super().__init__("safety_control_system")
        self.get_logger().info("Safety System Started")
        self.publisher_ = self.create_publisher(Bool, "safety_check", 10)
        self.create_timer(0.01,self.check_for_slip)
        self.slip = Bool()
        self.slip.data = True
    
    def check_for_slip(self):
        # code to identify person
        if self.slip.data:
            self.complaint_mode_activate()
            
    def complaint_mode_activate(self): #will replace this with the machine learning code
        self.publisher_.publish(self.slip)



def main(args=None):
    rclpy.init(args=args)
    node = CheckForSlipNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
