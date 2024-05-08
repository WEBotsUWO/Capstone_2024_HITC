import milestone_arm_only as m
import numpy as np


def getJointAngles2D(cartesian_trajectory, current_joint_angles):

    '''
    The goal of this function is to use accept an array of xy coordinates that from derived from the maze solving algorithm, convert these xy coordinates
    to joint angles, and also include the interpolated joint angles between coordinates.

    cartesian_trajectory: [[x1,y1],[x2,y2]....]
    current_joint_angles: [theta1 theta2 theta3] , for a 3 dof arm. home would be at [0 0 0]
    '''
    theta = current_joint_angles #initial configuration
    Time = 4 #time for each transition between points.
    joint_angles_trajectory = []
    catesian_points = []
    for point in cartesian_trajectory:
        catesian_points.append(cartesian_to_tranform(*point))

    trajectory = m.TrajectoryGenerator(catesian_points,Time) # the starting point of the robot's end-effector has to be considered. Trajectories in terms of transform functions.
    m.createCSV(trajectory,"path_planner/milestone2.csv")

def cartesian_to_tranform(x , y, z=0):
    '''
    This function assumes no rotation for the coordinates
    '''
    T = np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
    return  T


#testing Feedforward control
print("Testing Milestone 3 : FeedbackControl")
theta = [0,0,0.2]
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
m.traverseMaze(m.T_sci, m.T_scg, currConfiguration, m.T_sei, np.eye(6), np.eye(6)*20)




