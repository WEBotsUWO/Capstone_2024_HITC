#!/usr/bin/env python 3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32MultiArray
class PositionPublisher(Node):
    def __init__(self):
        super().__init__("position_publisher")
        self.get_logger().info("Position publisher started. Lets tell those joints what to do!")
        self.publisher_ = self.create_publisher(Float32MultiArray, "servo_pwms",10)
        self.timer_ = self.create_timer(0.5, self.callback_servo_publishing)
        self.counter_ = 2.0
        self.direction_ = 0.1
    def  callback_servo_publishing(self):
       
        percent_duty_cycle = Float32MultiArray()
        rounded_counter = round(self.counter_, 3)
        percent_duty_cycle.data = [rounded_counter, rounded_counter]
        self.counter_ += self.direction_
        if self.counter_ >= 4.0 or self.counter_ <= 2.0:
            self.direction_ *= -1
        self.publisher_.publish(percent_duty_cycle)
def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()