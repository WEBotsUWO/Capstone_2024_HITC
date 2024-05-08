#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from camera_capture import workspace_processing as wp
from camera_capture import RRT
from camera_capture import image_convert
from camera_capture import obstacleExtractor
from trajectory_generator.student_trajectory_algorithm import student_trajectory
from path_planner import modular_path_planner as m
from path_planner import modern_robotics as mr
import numpy as np
import math



# make robot object for testing

'''
------------------------------------------------------------------------------------------------------
State Publisher publishes the joints for both the 2 dof robot arm and the laser pointer based on the joint angles received from 
"joint_angle_trajectory" and "laser_angle_trajectory" respectively.

------------------------------------------------------------------------------------------------------
'''

class PlatformStatePublisherNode(Node):
    def __init__(self):
        super().__init__("state_publisher")
        self.get_logger().info("State_publisher node started now.")
        # mainpoints = wp.get_maze_details() #get start, goal, and grayscale image of workspace from  camera for 2 dof arm
        # start = mainpoints[0]
        # goal = mainpoints[1]
        # self.recieved_waypoints = student_trajectory.run_algorithm(start,goal,"/home/chris/capstone/hitc_ws/src/trajectory_generator/trajectory_generator/Converted_Maze.png") # perception planning of robot path.
        image_convert.updateMaze()
        obstacleExtractor.locate_obstacles()
        self.waypoint_list = RRT.solveMaze()
        # print(self.waypoint_list)
        # self.given_waypoints = [[450+120, -350],[450+120, -200]] #,[120 + 450 + 250, -200], [120 + 450 + 250, 200]] # in mm
        # self.recieved_waypoints = [[point[0]*0.001, point[1]*0.001] for point in self.given_waypoints] # in meters.

        self.recieved_waypoints = [[(-point[1] + (450 + 120)+461)*0.6*0.001, (-point[0]+329)*0.668*0.001] for point in self.waypoint_list]
        twoDofArm = m.myRobot()
        laserPointer = m.myRobot()

        '''
        Initialize twoDofArm
        '''
        thetaTwoDof = [0,0,0]
        baseL1 = 120 * 0.001
        L12 = 350 * 0.001
        L2e = 450 * 0.001
        M_0e = np.array([[1, 0, 0, baseL1+L2e],[0, 1, 0, -L12], [0, 0, 1, 0],[0, 0, 0, 1]]) # home configuration
        B = [] # screw axis    

        B1 = [0,0,1,L12,L2e,0] #joint screw axis
        B2 = [0,0,1,0,L2e,0]
        B3 = [0,0,1,0,0,0]
        B = [B1,B2,B3]

        twoDofArm.setup(M_0e,B)

        '''
        -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        Initialize laser pointer

        instead of drawing wrtiing out the screw axis the angles of the servo motors are simple enough to calculate  by hand (or computer with simple algorithms).
        -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        '''

        # thetaLaser = [0,0,0,0,0] # Current Configuration
        # home_config = np.array([[1, 0, 0, 0],[0, 1, 0, -53.81*0.001], [0, 0, 1, 0],[0, 0, 0, 1]])
        # B1 = [0,0,1,0,-53.81*0.001,0] #joint screw axis
        # B2 = [0,1,0,0,-360*0.001,53.81*0.001]
        # B3 = [0,0,0,0,0.1478,0.9890] # v3 = [0, -sin(8.5 degrees), -cos(8.5 degrees)]
        # B4 = [0,0,1,0,0,0]
        # B5 = [1,0,0,0,0,0]
        # body_screw_axis = [B1,B2,B3,B4,B5]

        # laserPointer.setup(home_config,body_screw_axis)
        # waypoints = [[point[0]*0.001, point[1]*0.001] for point in waypoints] # in meters.
        waypoints_transforms = []
        for point in self.recieved_waypoints:
            waypoints_transforms.append(twoDofArm.coordinatesToTransforms(*point))
        pathname = "/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_angle_trajectory.csv"
        # pathname = "/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/twoDofArm_moreSamples.csv"
        # Tgoal = np.array([[1, 0, 0, 79.1*0.001],[0, 1, 0, -200*0.001], [0, 0, 1, 0],[0, 0, 0, 1]])
        # waypoints_transforms = [home_config, Tgoal]
        twoDofArm.traverseGivenPath(waypoints_transforms, thetaTwoDof, np.eye(6), np.eye(6)*20, pathname)
        # twoDofArm.traverseGivenPath(self.recieved_waypoints, thetaTwoDof, np.eye(6), np.eye(6)*20, '/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_angle_trajectory.csv')
        # laserPointer.traverseGivenPath(self.recieved_waypoints, thetaLaser, np.eye(6), np.eye(6)*20, '/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/laser_angle_trajectory.csv')



        self.joint_angle_trajectory = []
        # When node starts the trajectory must already be in the working directory.  We will then put csv trajectory into an array and send each waypoint to the robot later.
        with open('/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_angle_trajectory.csv', newline='') as csvfile:
                file1 = open('/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_trajectory_other.txt', 'w')
                L = []
                csvreader = csv.reader(csvfile, delimiter=',')
                L.append('{\n')
                for row in csvreader:
                    self.joint_angle_trajectory.append(row)
                    L.append('{'+row[0]+', '+row[1]+'},\n')
                L.append('},\n')
                file1.writelines(L) 
                # Checking if the data is
                # written to file or not
                file1 = open('/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_trajectory.txt', 'r')
                print(file1.read())
                file1.close()

        self.waypoint_index = 0 # Current waypoint index. for 2 dof arm

        '''
         For each of the robot's joints we will calculate the forward kinematics, extract the x and y coordinates, and then obtain the angles of 
         for our servo motors (the laser).
        '''



        self.laser_angle_trajectory = []
        theta = [0.0 , 0.0, 0.0]
        for configuration in self.joint_angle_trajectory:
            theta[0] = float(configuration[0])
            theta[1] = float(configuration[1])
            theta[2] = float(configuration[2])
            # print(theta)
        
            # print(twoDofArm.M_0e)
            # print(twoDofArm.B)
            transformationMatrix = twoDofArm.getT_0e(twoDofArm.M_0e, theta, twoDofArm.B)

            # print(transformationMatrix)
            x = transformationMatrix[0][3] - (450+120-79.1)*0.001 #subtracting difference in x refrence frame and robot arm.
            y = transformationMatrix[1][3] #no difference in y
            print("\n")
            print(x,y)
            print("\n")
            servo1_theta = 0
            servo2_theta = 0
            
            a = math.sqrt(x**2 + y**2)

            if y < 0:
                servo1_theta = math.atan(x/abs(y)) * 57.2958
            elif y  > 0:
                servo1_theta = 180 - (math.atan(x/abs(y)) * 57.2958)
            elif y == 0:
                servo1_theta = 90
            servo2_theta = math.atan(a/(360.33*0.001))* 57.2958

            self.laser_angle_trajectory.append([servo1_theta,servo2_theta])

        # with open('/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/laser_angle_trajectory.csv', newline='') as csvfile:
        #         csvreader = csv.reader(csvfile, delimiter=',')
        #         for row in csvreader:
        #             self.laser_angle_trajectory.append(row)


        # self.laser_waypoint_index = 0


        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.publisher_micro_ros = self.create_publisher(Float32MultiArray, '/student_trajectory',10)
        self.subscriber_ = self.create_subscription(Float32MultiArray,'/encoder_readings', self.publish_readings,10)
        self.timer = self.create_timer(0.001, self.timer_publish_angles_to_esp)
        
    def timer_publish_angles_to_esp(self): #should send jointangles to esp32 (currently still sends directly to robot simulation)

        ''' function should send both the joints for the 2 dof robot arm and the laser pointer'''
        
        motor_joints = Float32MultiArray()

        # 2 dof joint angles 

        bl1 = float((self.joint_angle_trajectory[self.waypoint_index][0]))
        l12 = float((self.joint_angle_trajectory[self.waypoint_index][1]))
        l2e = float((self.joint_angle_trajectory[self.waypoint_index][2])) #ignoring. Initially sent to simulation, but no actual actuator for microros.       

        # laser pointer

        servo1 = float((self.laser_angle_trajectory[self.waypoint_index][0]))
        servo2 = float((self.laser_angle_trajectory[self.waypoint_index][1]))

        # motor_joints.data = [bl1,l12, servo1, servo2] #  for sending both the 2 dof arm and the laser pointer angles.
        # motor_joints.data = [bl1,l12,l2e]
        motor_joints.data = [servo1, servo2] 

        if (self.waypoint_index < len(self.joint_angle_trajectory)-1):
            self.waypoint_index += 1 
    
        self.publisher_micro_ros.publish(motor_joints)

        encoder_readings = Float32MultiArray()
        encoder_readings.data = [bl1,l12,l2e]
        self.publish_readings(encoder_readings)

         

    def publish_readings(self, msg): # after recieving the encoder's angles then send these angles to the robot simulation (subscriber callback). This works!!!!
       # Configuring joint message to be sent
        # Update joint state
        simulation_joints = JointState()
        now = self.get_clock().now()
        simulation_joints.header.stamp = now.to_msg()
        simulation_joints.name = ['base_L1', 'L1-L2','L2-EE'] 
        bl1 = msg.data[0]
        l12 = msg.data[1]
        # l2e = float((self.results[self.waypoint_index][2]))

        simulation_joints.position = [bl1,l12, 0.0] # ignoring end-effector orientation for simulation. May try to get same end-effector from micro controller... later
        self.publisher_.publish(simulation_joints)



    
def main(args=None):
    rclpy.init(args=args)
    node = PlatformStatePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
