import milestone_arm_only as m
import numpy as np

# testing NextState function
print("Testing Milestone 1 : NextState functon")

theta = [0,0,0] # joint angles
theta_dot = [1, 1, 1] # joint speeds

current_configuration = theta
joint_speeds = theta_dot   
joint_speed_limits = [5,5,5]  # rad/s
timestep = 0.01

# for i in range(100):
next_configuration = m.NextState(current_configuration,joint_speeds,timestep,joint_speed_limits)
current_configuration = next_configuration
print(current_configuration)

# #testing specific instance of robot motion.
# wheel_joint_speeds = np.array([25.139,25.139,25.139,25.139,-0.,15.385,-24.823,9.438,0.])
# wheel_joint_speed_limits = [5,5,5,5,10, 10, 10, 10, 10]
# current_configuration =[0.000000e+00,6.967800e+01,-0.000000e+00,0.000000e+00,-1.519800e-01,3.270000e-02,2.922000e-02,0.000000e+00,1.464495e+01,1.464495e+01,1.464495e+01,1.464495e+01]
# next_configuration = m.NextState(current_configuration,wheel_joint_speeds,timestep,wheel_joint_speed_limits)
# print("next instance : ", next_configuration) 



print("Testing appendToMovementList")
configurationList = []
gripper = 1

steps = 800
time = (steps+100)*0.01 #time has to be slightly higher than steps. 8 seconds for each segment.
m.appendToMovementList(configurationList, m.T_sei,m.T_sci, time, steps, gripper)
# m.createCSV(configurationList,"path_planner/timmingTest.csv")
#testing TrajectoryGenerator
print("Testing Milestone 2 : TrajectoryGenerator")

Time = 4 #time for each transition between points.

trajectory = m.TrajectoryGenerator([m.T_sei, m.T_sci, m.T_scg],Time)
m.createCSV(trajectory,"/home/chris/capstone/hitc_ws/src/path_planner/path_planner/milestone2.csv")


#testing Feedforward control
print("Testing Milestone 3 : FeedbackControl")
theta = [0,0,0]
X_d = np.array([[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
X_d_next = np.array([[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]])
X = m.getX(m.M_0e,theta, m.B)
K_p = np.zeros(6)
K_i = np.zeros(6)
delta_t = 0.01

jacobian2 = m.getJacobian(theta)

correctionTwist,_ = m.FeedbackControl(X,X_d,X_d_next,K_p,K_i,delta_t)
# print("V: ", correctionTwist)

# print("Jacobian 1 : \n ", jacobian)
print("Jacobian 2 : \n ", jacobian2)

joint_speeds = m.end_twist_to_joint_speeds(jacobian2, correctionTwist)


#Testing Final project
print("Testing Final step : Completing the Project and Your Submission")

theta = [0,0,0] # joint angles
currConfiguration = theta
baseL1 = 120 * 0.001
L12 = 350 * 0.001
L2e = 450 * 0.001
start_point = np.array([[1, 0, 0, baseL1+L2e],[0, 1, 0, -L12], [0, 0, 1, 0],[0, 0, 0, 1]])
end_point = np.array([[1, 0, 0, 0.001*(450+175+120)],[0, 1, 0, -303.108*0.001], [0, 0, 1, 0],[0, 0, 0, 1]])
m.traverseMaze(start_point, m.T_scg, [m.T_sci], currConfiguration, np.eye(6), np.eye(6)*20)




