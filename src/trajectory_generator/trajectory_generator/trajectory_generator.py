#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_generator.student_trajectory_algorithm import student_trajectory
from example_interfaces.msg import Float32MultiArray
import numpy as np
# from path_planner
'''
------------------------------------------------------------
Given the workspace in "Converted_Maze.npy" (see Converted_Maze.png)
come up with an algorithm to traverse the the workspace. Make sure to save 
the final trajectory in the global variable called 'Waypoints'
------------------------------------------------------------
'''

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__("trajectory_generator")
        self.get_logger().info("trajectory_generator node started. Sending student's trajectory...")
        self.publisher_ = self.create_publisher(Float32MultiArray, 'trajectory', 10)
        self.timer_ = self.create_timer(1,self.publish_waypoints)
        start = np.array([1228.864, 107.367])
        goal = np.array([311.630, 800.650])
        self.waypoints = student_trajectory.run_algorithm(start,goal,"/home/chris/capstone/hitc_ws/src/trajectory_generator/trajectory_generator/Converted_Maze.png") # perception planning of robot path.
         # gets x and y positions for traversing the maze
        self.counter = 0
        #use modern robotics to create final trajectory of joint positions. # call service ?
        
        self.curr_joint_trajectory = Float32MultiArray()
    
    def publish_waypoints(self):
        if self.counter < len(self.waypoints):
            self.curr_joint_trajectory.data = list(self.waypoints[self.counter])    # the list should be changed to the list of joint configurations. After I have made use of the modern_robotics library.
            self.publisher_.publish(self.curr_joint_trajectory)
            self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
