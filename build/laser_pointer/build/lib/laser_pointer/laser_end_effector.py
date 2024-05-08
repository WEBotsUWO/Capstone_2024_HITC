#!/usr/bin/env python 3
import rclpy
import lgpio 
from rclpy.node import Node
from std_msgs.msg import Bool
class LaserEndEffector(Node):
    def __init__(self):
        super().__init__("laser_end_effector")
        self.get_logger().info("Lazer is ready")
        self.subscriber_ = self.create_subscription(Bool, "laser_on", self.callback_laser_on,10)
        self.h = lgpio.gpiochip_open(0)
        self.laser = 24
        lgpio.gpio_claim_output(self.h,self.laser)
        self.currentState = 0

    def callback_laser_on(self, laser_on):
        if laser_on.data:
            lgpio.gpio_write(self.h, self.laser, 1 )
        else:
            lgpio.gpio_write(self.h, self.laser, 0)
        
def main(args=None):
    rclpy.init(args=args)
    node = LaserEndEffector()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()