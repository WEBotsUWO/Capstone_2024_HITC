import modular_path_planner as m
import numpy as np


# make robot object for testing



# start = [((450 + 120)* 0.001), (-200* 0.001)]
# goal =  [((450 + 120 + 250)* 0.001), (200 * 0.001)]
# baseL1 = 120 * 0.001
# L12 = 350 * 0.001
# L2e = 450 * 0.001

# home_config = np.array([[1, 0, 0, baseL1+L2e],[0, 1, 0, -L12], [0, 0, 1, 0],[0, 0, 0, 1]]) ;
# B1 = [0,0,1,L12,L2e,0] #joint screw axis
# B2 = [0,0,1,0,L2e,0]
# B3 = [0,0,1,0,0,0]
# body_screw_axis = [B1,B2,B3]

# myRobo.setup(home_config, body_screw_axis)
# # testing NextState function
# print("Testing Milestone 1 : NextState functon")

# theta = [0,0,0] # joint angles
# theta_dot = [1, 1, 1] # joint speeds

# current_configuration = theta
# joint_speeds = theta_dot   
# joint_speed_limits = [5,5,5]  # rad/s
# timestep = 0.01

# # for i in range(100):
# next_configuration = myRobo.NextState(current_configuration,joint_speeds,timestep,joint_speed_limits)
# current_configuration = next_configuration
# print(current_configuration)

# # #testing specific instance of robot motion.
# # wheel_joint_speeds = np.array([25.139,25.139,25.139,25.139,-0.,15.385,-24.823,9.438,0.])
# # wheel_joint_speed_limits = [5,5,5,5,10, 10, 10, 10, 10]
# # current_configuration =[0.000000e+00,6.967800e+01,-0.000000e+00,0.000000e+00,-1.519800e-01,3.270000e-02,2.922000e-02,0.000000e+00,1.464495e+01,1.464495e+01,1.464495e+01,1.464495e+01]
# # next_configuration = m.NextState(current_configuration,wheel_joint_speeds,timestep,wheel_joint_speed_limits)
# # print("next instance : ", next_configuration) 



# # print("Testing appendToMovementList")
# # configurationList = []
# # gripper = 1

# # steps = 800
# # time = (steps+100)*0.01 #time has to be slightly higher than steps. 8 seconds for each segment.
# # myRobo.appendToMovementList(configurationList, m.T_sei,m.T_sci, time, steps, gripper)
# # # m.createCSV(configurationList,"path_planner/timmingTest.csv")
# # #testing TrajectoryGenerator
# print("Testing Milestone 2 : TrajectoryGenerator")

# # Time = 4 #time for each transition between points.
# # waypoints = [m.T_sei, m.T_sci, m.T_scg]
# # trajectory = myRobo.TrajectoryGenerator(waypoints,Time)
# # myRobo.createCSV(trajectory,"/home/chris/capstone/hitc_ws/src/path_planner/path_planner/milestone2.csv")


# #testing Feedforward control
# print("Testing Milestone 3 : FeedbackControl")
# theta = [0,0,0]
# X_d = np.array([[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
# X_d_next = np.array([[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]])

# # print("Argument 1: " , myRobo.M_0e)
# # print("Argument 2: " , theta)
# # print("Argument 3: " , myRobo.B)


# X = myRobo.getX(myRobo.M_0e,theta, myRobo.B)
# K_p = np.zeros(6)
# K_i = np.zeros(6)
# delta_t = 0.01

# jacobian2 = myRobo.getJacobian(theta)

# correctionTwist,_ = myRobo.FeedbackControl(X,X_d,X_d_next,K_p,K_i,delta_t)
# # print("V: ", correctionTwist)

# # print("Jacobian 1 : \n ", jacobian)
# print("Jacobian 2 : \n ", jacobian2)

# joint_speeds = myRobo.end_twist_to_joint_speeds(jacobian2, correctionTwist)


#Testing Final project
print("Testing Final step : Completing the Project and Your Submission")

twoDofArm = m.myRobot()
laser = m.myRobot()
# 2 dof robot
theta = [0,0,0] # Current Configuration
currConfiguration_twoDof = theta
baseL1 = 120 * 0.001
L12 = 350 * 0.001
L2e = 450 * 0.001
start_point = np.array([[1, 0, 0, baseL1+L2e],[0, 1, 0, -L12], [0, 0, 1, 0],[0, 0, 0, 1]])
end_point = np.array([[1, 0, 0, 0.001*(450+175+120)],[0, 1, 0, -303.108*0.001], [0, 0, 1, 0],[0, 0, 0, 1]])


# laser pointer
theta = [0,0,0,0,0] # Current Configuration
currConfiguration = theta
home_config = np.array([[1, 0, 0, 0],[0, 1, 0, -53.81*0.001], [0, 0, 1, 0],[0, 0, 0, 1]])
B1 = [0,0,1,0,-53.81*0.001,0] #joint screw axis
B2 = [0,1,0,0,-360*0.001,53.81*0.001]
B3 = [0,0,0,0,0.1478,0.9890] # v3 = [0, -sin(8.5 degrees), -cos(8.5 degrees)]
B4 = [0,0,1,0,0,0]
B5 = [1,0,0,0,0,0]
body_screw_axis = [B1,B2,B3,B4,B5]

laser.setup(home_config,body_screw_axis)
waypoints = [[450+120, -350],[450+120, -200],[120 + 450 + 250, -200], [120 + 450 + 250, 200]] # in mm
# waypoints = [[79.1+150, 0],[79.1+300, 200]] # in mm

waypoints = [[point[0]*0.001, point[1]*0.001] for point in waypoints] # in meters.
waypoints_transforms = []
for point in waypoints:
    waypoints_transforms.append(twoDofArm.coordinatesToTransforms(*point))
# pathname = "/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_angle_trajectory.csv"
pathname = "/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/twoDofArm_moreSamples.csv"
# Tgoal = np.array([[1, 0, 0, 79.1*0.001],[0, 1, 0, -200*0.001], [0, 0, 1, 0],[0, 0, 0, 1]])
# waypoints_transforms = [home_config, Tgoal]
twoDofArm.traverseGivenPath(waypoints_transforms, currConfiguration_twoDof, np.eye(6), np.eye(6)*20, pathname)
# print(twoDofArm.getT_0e(twoDofArm.M_0e,[1.34728,0.11801,-0.01761,-1.34876,0.11508],body_screw_axis))
# m.traverseMaze(start_point, m.T_scg, [m.T_sci], currConfiguration, np.eye(6), np.eye(6)*20)
    


