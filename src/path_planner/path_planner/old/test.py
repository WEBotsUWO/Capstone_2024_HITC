import milestone as m
import numpy as np

# testing NextState function
print("Testing Milestone 1 : NextState functon")
wheel_angles = [0,0,0,0] #wheel angles in radian
theta = [0,0,0,0,0] # joint angles
q = [0,0,0] # chassis configuration [phi, x, y]
u = [10, 10, 10, 10] #wheel velocities in radians/s

theta_dot = [0, 0, 0, 0, 0]

current_configuration = np.hstack((q,theta,wheel_angles))
wheel_joint_speeds = np.hstack((u,theta_dot))   
timestep = 0.01
wheel_joint_speed_limits = [10,10,10,10,10,5,5,5,5]

for i in range(100):
      
    next_configuration = m.NextState(current_configuration,wheel_joint_speeds,timestep,wheel_joint_speed_limits)
    current_configuration = next_configuration
print(current_configuration)

#testing specific instance of robot motion.
wheel_joint_speeds = np.array([25.139,25.139,25.139,25.139,-0.,15.385,-24.823,9.438,0.])
wheel_joint_speed_limits = [5,5,5,5,10, 10, 10, 10, 10]
current_configuration =[0.000000e+00,6.967800e+01,-0.000000e+00,0.000000e+00,-1.519800e-01,3.270000e-02,2.922000e-02,0.000000e+00,1.464495e+01,1.464495e+01,1.464495e+01,1.464495e+01]
next_configuration = m.NextState(current_configuration,wheel_joint_speeds,timestep,wheel_joint_speed_limits)
print("next instance : ", next_configuration) 


print("Testing appendToMovementList")
configurationList = []
gripper = 1

steps = 800
time = (steps+100)*0.01 #time has to be slightly higher than steps. 8 seconds for each segment.
m.appendToMovementList(configurationList, m.T_sei,m.T_sci, time, steps, gripper)
m.createCSV(configurationList,"path_planner/timmingTest.csv")
#testing TrajectoryGenerator
print("Testing Milestone 2 : TrajectoryGenerator")

Time = 4 #time for each transition between points.

trajectory = m.TrajectoryGenerator(m.T_sei, m.T_sci, m.T_scg, m.T_ce_grasp, m.T_ce_standoff,Time)
m.createCSV(trajectory,"path_planner/milestone2.csv")



#testing Feedforward control
print("Testing Milestone 3 : FeedbackControl")
theta = [0,0,0.2,-1.6,0]
X_d = np.array([[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
X_d_next = np.array([[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]])
X = m.getX(m.T_sb([0,0,0]), m.T_b0, m.M_0e,theta, m.B)
K_p = np.zeros(6)
K_i = np.zeros(6)
delta_t = 0.01

jacobian = np.array([[0.030, -0.030, -0.030, 0.030, -0.985, 0, 0, 0, 0],
                     [0,      0,      0,     0,      0,    -1,-1,-1, 0],
                     [-0.005, 0.005,  0.005, -0.005, 0.170, 0, 0, 0, 1],
                     [0.002,  0.002,  0.002, 0.002,  0,    -0.240, -0.214, -0.218, 0],
                     [-0.024, 0.024,  0,     0,      0.221, 0,    0,  0, 0],
                     [0.012,  0.012,  0.012, 0.012,  0,    -0.288, -0.135, 0, 0]])


jacobian2 = m.getJacobian(theta)

correctionTwist,_ = m.FeedbackControl(X,X_d,X_d_next,K_p,K_i,delta_t)
# print("V: ", correctionTwist)

# print("Jacobian 1 : \n ", jacobian)
# print("Jacobian 2 : \n ", jacobian2)

joint_speeds = m.end_twist_to_joint_speeds(jacobian2, correctionTwist)

# print("Joint speeds : ", joint_speeds)


#testing Final step
print("Testing Final step : Completing the Project and Your Submission")

wheel_angles = [0,0,0,0] #wheel angles in radian
theta = [-1,5,0.2,-1.6,0] # joint angles
q = [0,-1,0.5] # chassis configuration

currConfiguration = np.hstack((q,theta,wheel_angles))
m.moveBlockOperation(m.T_sci, m.T_scg, currConfiguration, m.T_sei, np.eye(6), np.eye(6)*20)