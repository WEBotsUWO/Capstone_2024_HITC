import modern_robotics as mr
import numpy as np


start = [((450 + 120)* 0.001), (-200* 0.001)]
goal =  [((450 + 120 + 250)* 0.001), (200 * 0.001)]
baseL1 = 120 * 0.001
L12 = 350 * 0.001
L2e = 450 * 0.001

M_0e = np.array([[1, 0, 0, baseL1+L2e],[0, 1, 0, -L12], [0, 0, 1, 0],[0, 0, 0, 1]])
B1 = [0,0,1,L12,L2e,0] #joint screw axis
B2 = [0,0,1,0,L2e,0]
B3 = [0,0,1,0,0,0]
B = [B1,B2,B3]

T_sei = M_0e # initial position of end-effector with respect to reference frame
T_sci = np.array([[1,0,0,start[0]], [0,1,0,start[1]], [0,0,1,0], [0,0,0,1]]) # start position 
T_scg = np.array([[1,0,0,goal[0]], [0,1,0,goal[1]], [0,0,1,0],[0,0,0,1]]) # end position
# T_ce_standoff = np.array([[0.169967, 0.0, 0.98545,0], [0,1,0,0], [-0.98545, 0.0, 0.169967,0.3], [0,0,0,1]])
# T_ce_grasp = np.array([[-np.cos(45),0,np.cos(45),0.005], [0,1,0,0], [-np.sin(45),0,-np.sin(45),0], [0,0,0,1]])



'''
--------------------------------------------------------------------------------------------------------
MILESTONE 1 : NextState, wheelspeed_to_chassisvelocity, chassisVelocity_to_unitStepChassisConfiguration
get_within_speed_limit
-------------------------------------------------------------------------------------------------------
'''
def NextState(current_configuration, joint_speed, timestep, joint_speed_limits):
    '''
    Given the input above (the current state and limits) this function should output the next configuration after the timestep.
    '''
    # for the duration of this function the wheel and joint speeds are assumed to be constant.
    # joint_speed = wheel_joint_speeds[4:9]
    # joint_speed_limits = wheel_joint_speed_limits[4:9] #  function should be modified for just arm joints
    curr_arm_speeds =  get_within_speed_limit(joint_speed, joint_speed_limits)
    curr_arm_config = current_configuration #  function should be modified for just arm configuration.

    new_arm_config = np.add(curr_arm_config, [i* timestep for i in curr_arm_speeds])
    current_configuration = new_arm_config 
    return np.around(current_configuration,6)

def get_within_speed_limit(given_speeds, given_limits):
    proper_speeds = []
    for index in range(len(given_speeds)):
        if abs(given_speeds[index]) <= abs(given_limits[index]):
            proper_speeds.append(given_speeds[index])
        else:
            proper_speeds.append((given_speeds[index]/abs(given_speeds[index]))*given_limits[index])
    # print("Proper speeds : ", proper_speeds)
    return proper_speeds

'''
-----------------------------------------------------------------------------------------------------
MILESTONE 2 : 
-----------------------------------------------------------------------------------------------------
'''


def TrajectoryGenerator(points,Time):

    '''
    points : [T_sei, T_sci, T_scg]
    T_sei : initial configuration of the end-effector to the reference trajectory
    T_sci : cube's initial configuration
    T_scg : final/goal configuration of  cube
    T_ce_grasp : end-effector configuration relative to the cube when it is grasping the cube
    T_ce_standoff: The configuration of the end-effector {e} relative to the cube  frame  before lowering to grasp (and after releasing)
    k : number of trajectory reference configurations per 0.01 (timestep) seconds k >=1 for entire trajectory
    T : total time for entire trajectory
    '''
    #k #number of reference configurations per 0.01 seconds
    K_each = Time*100 
    T_each = (K_each+100)*0.01
    transistionStep = 0.630 * 100 #about 0.625 seconds
    transistionTime = (transistionStep + 100)*0.01
    # point1 = T_sei #start configuration for this segment
    # point2 = T_sci #start configuration for this segment
    # point3 = T_scg #goal position
    
    listOfConfigurations = points
    listOfGripperStates = [1, 1, 1]
    N = np.around(listOfConfigToTrajectory(listOfConfigurations,T_each, K_each, listOfGripperStates, transistionTime,transistionStep),6)
    # createCSV(N, "CapstoneProject/milestone2.csv")
    return N # where each line is of the form : r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state 
def listOfConfigToTrajectory(configList, Time, Steps,gripper, transistionTime,transistionStep):
    '''
    Given a list of configurations (points for the end-effector to visit) the robot moves through each 
    configuration with the number of Steps indicated. The time between each configuration (not sub steps) 
    is the interstep.

    The gripper of the robot can be open or closed so we have an array called gripper that has only 1's and 0's
    corresponding to the state of the gripper at the corresponding configuration.
    '''
    Trajectory = []
    for i in range(1,len(configList)):
            if gripper[i-1] != gripper[i]:
                appendToMovementList(Trajectory, configList[i-1], configList[i], Time, Steps,gripper[i-1])
                appendToMovementList(Trajectory, configList[i], configList[i], transistionTime, transistionStep,gripper[i])
            else:
                
                appendToMovementList(Trajectory, configList[i-1], configList[i], Time, Steps,gripper[i])
    return Trajectory

def make_configuration_a_row(T):
    T_row  = []
    p_list = []
    # Of the form r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz
    for row in T:
        p = row[-1]
        r = np.array(row[:-1])
        r = np.around(r,3).tolist() # rounding elements to 3 dp
        T_row.extend(r)
        p_list.append(p)
    T_row.extend(p_list)
    return T_row

def make_row_a_configuration(row):
    '''
    row is of the form : r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz
    '''
    T = []
    T.append([*row[:3],row[9]])
    T.append([*row[3:6],row[10]])
    T.append([*row[6:9],row[11]])
    T.append([0,0,0,1])

    return T

def appendToMovementList(currTrajectoryList, p1, p2, Time, Steps, gripper):
    '''
    currTrajectoryList : the universal list of joint trajectories where each row is of the form
         => r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
    p1 : current configuration that the endeffecto is at (point 1)
    p2 : next configuration to create a path to (point 2)
    Time : Total time that should be taken from point 1 to point 2 (rest to rest)
    Steps : The total number of points (intermidiate steps) from point 1 to point 2
    '''
   
    setOfConfigurations = mr.CartesianTrajectory(p1,p2,Time,Steps,5)
    for  configuration in setOfConfigurations:
        formated_config = make_configuration_a_row(configuration[:-1]) #exluding the last row "[0, 0, 0, 1]"
        formated_config.append(gripper)
        currTrajectoryList.append(formated_config)
    pass
    

def createCSV(givenVector,filename="./capstone.csv", headers=None):
    # for vector in givenVector:
        # for i in range(len(vector)):
        #     num = round(vector[i],3)
        #     if (i + 1) == len(vector):
        #         print(num,end = "\n")
        #     else:
        #         print(num,end = ", ")

    import csv
    with open(filename, 'w',newline='') as csvfile:
        csvwriter = csv.writer(csvfile)

        if headers != None:
            csvwriter.writerow(headers)
        csvwriter.writerows(givenVector)
    return

def appendToCSV(givenVector,filename):
    # for vector in givenVector:
        # for i in range(len(vector)):
        #     num = round(vector[i],3)
        #     if (i + 1) == len(vector):
        #         print(num,end = "\n")
        #     else:
        #         print(num,end = ", ")

    import csv
    with open(filename, 'a') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(givenVector)
    return


'''
---------------------------------------------------------------------------------------
MILESTONE 3 : Feedforward Control
---------------------------------------------------------------------------------------
'''
def FeedbackControl(X, X_d, X_d_next, K_p, K_i, delta_t):

    '''

    X : actual end-effector configuration T_se (function of q and theta)
    X_d : current end-effector reference configuration T_se,d 
    X_d_next : end-effector reference configuration at the next timestep in the reference trajectory T_se,d,next
    K_p : proportional gain
    K_i : integral gain
    delta_t : timestep between reference trajectory configurations.

    X_err : error twist that takes the end_effector from X to X_d
    Adj_Xinv_Xd : The adjoint matrix that will change the reference frame of twist V_d from frame d to actual frame of endeffector e (X is in frame e)
    V_d_vector : 
    '''

    Adj_Xinv_Xd = mr.Adjoint(np.matmul(mr.TransInv(X), X_d)) 
    X_err_matrix = mr.MatrixLog6(np.matmul(mr.TransInv(X), X_d))
    X_err_vector = mr.se3ToVec(X_err_matrix)
    V_d_matrix = mr.MatrixLog6(np.matmul(mr.TransInv(X_d), X_d_next)) / delta_t # dividing by delta_t becuase the log gives us S_theta but we want S_theta-dot
    V_d_vector = mr.se3ToVec(V_d_matrix)
    X_err_intergral = X_err_vector*delta_t

    
    correctionTwist = np.matmul(Adj_Xinv_Xd, V_d_vector) + np.matmul(K_p,X_err_vector) + np.matmul(K_i,X_err_intergral)
    # print("V_d : ", V_d_vector)
    # print("Ad_Xinv_Xd * Vd : ", np.matmul(Adj_Xinv_Xd, V_d_vector))
    # print("V : ", correctionTwist)
    # print("X_err : ", X_err_vector)    
    return (np.around(correctionTwist,5),X_err_vector)

def getX(M_0e,theta,B):
    currX = getT_0e(M_0e,theta,B) # theta here could be fed in through sensor data
    return currX # Current position of end effector.
def end_twist_to_joint_speeds(jacobian, twist):

    jointSpeeds = np.around(np.matmul(np.linalg.pinv(jacobian), twist),3)

    return jointSpeeds
def getJacobian(thetalist):
    '''
    https://www.coursera.org/learn/modernrobotics-course2/lecture/JsioE/body-jacobian-chapter-5-1-2-through-5-1-4

    B : this is the matrix of screw axes with respect to the endeffector. This will be used to get the body jacobian.
    There is already a function for this in the mr library 
    '''
    # body jacobian
    # print("screw axis : \n", np.array(B).T)
    Jarm = mr.JacobianBody(np.array(B).T, thetalist)
    
    # print(Jchasis, Jarm)
    Je = Jarm
    Je = np.around(Je,5)
    # print(Je)
    return Je
def getT_0e(M_0e,theta,B):
    '''
    Function that get the forward kinematics of the endeffector with the given joint angles and screwaxis
    '''
    T_0e = M_0e
    for index in range(len(theta)):
        T_0e = np.matmul(T_0e, mr.MatrixExp6(mr.VecTose3(B[index])*theta[index]))
    return T_0e
def isJacobianSingular(jacobian):
    pass  


'''
-------------------------------------------------------------------------------------
FINAL STEP : Completeing the Project 
-------------------------------------------------------------------------------------
'''
# def traverseMaze(T_sci, T_scg, currConfiguration, T_sei, Kp, Ki):
def traverseMaze(start_Transform, endTransform, remaining_waypoint_Tranforms, currConfiguration, Kp, Ki):
    '''
    - first need to create the trajectory
    - use our control system to follow the trajectory
    - From the twists that we get from our contol system we derive the joint and wheel speeds.
    - using new speeds and time passed get the next state.
    - repeat until robot has traveresed the trajectory
    '''

    fullConfigurationList = []
    error = []
    Time = 5 #seconds between each point in the transition
    points = [start_Transform, *remaining_waypoint_Tranforms, endTransform]
    refTrajList = TrajectoryGenerator(points,Time) #first need to create the trajectory

    current_configuration = currConfiguration
    theta_speed_limit = [10,10,10]
    joint_speed_limits = theta_speed_limit

    for i in range(1,len(refTrajList)):
        theta = current_configuration
        X = np.around(getX(M_0e,theta, B),6)
        X_d = np.around(make_row_a_configuration(refTrajList[i-1]),6)
        X_d_next = np.around(make_row_a_configuration(refTrajList[i]),6)
        K_p = Kp
        K_i = Ki
        delta_t = 0.01

        commandTwist, X_error = FeedbackControl(X,X_d,X_d_next,K_p, K_i,delta_t)    # use our control system to follow the trajectory
        error.append(X_error)
        jacobian = getJacobian(theta)
        joint_speeds = end_twist_to_joint_speeds(jacobian,commandTwist) # From the twists that we get from our contol system we derive the joint
        fullConfiguration = [*current_configuration, refTrajList[i-1][-1]] # joint angles and gripper state(refTrajList[i-1][-1])
        fullConfigurationList.append(fullConfiguration)      
        next_configuration = NextState(current_configuration,joint_speeds,delta_t,joint_speed_limits) # using new speeds and time passed get the next state.
        current_configuration = next_configuration
    print("Creating CSV animation")
    createCSV(fullConfigurationList,"/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_angle_trajectory.csv")
    print("Generating error plot data")
    createCSV(error,"./error.csv", ["W_x_error","W_y_error","W_z_error","V_x_error","V_y_error","V_z_error"])
    print("Done.")
    return fullConfigurationList
    