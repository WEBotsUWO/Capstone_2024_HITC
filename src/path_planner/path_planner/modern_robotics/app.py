
import core as mr
import numpy as np

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """
    ---------------------------------------------------------------------
    Computes inverse kinematics in the body frame for an open chain robot
    ---------------------------------------------------------------------
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

    """
    --------------------------------------------------------------------------------------
    "jointVectorIterations" is an array created to hold all the iterations of joint angles
    --------------------------------------------------------------------------------------
    """
    jointVectorIterations = np.array([])
    
    while err and i < maxiterations:
        omgMag = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        linearMag = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        
        """
        --------------------------------------------------------------------------------------
        The function that outputs the log file, "displayReport()", also returns 
        and updates the jointVectorIterations 
        --------------------------------------------------------------------------------------
        """
        jointVectorIterations = displayReport(i,thetalist,T, Vb , omgMag, linearMag, jointVectorIterations)
        
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        i = i + 1
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    """
    --------------------------------------------------------------------------------------
    "createCSV" is a function created generate a CSV formatted output.
    --------------------------------------------------------------------------------------
    """
    createCSV(jointVectorIterations)
    return (thetalist, not err)


def createCSV(givenVector):
    for vector in givenVector:
        for i in range(len(vector)):
            num = round(vector[i],4)
            if (i + 1) == len(vector):
                print(num,end = "\n")
            else:
                print(num,end = ",")
    return
def  displayReport(iteration,jointVector,SE3, errorTwist,angularError, linearError, jointVectorIterations):
    print("\n")
    print("Iteration : ", iteration)
    print("Joint Vector : ", jointVector)
  
    if(jointVectorIterations.size > 0):
        print("adding another vector...")
        jointVectorIterations = np.vstack([jointVectorIterations,jointVector])
    else:
        print("Adding first iteration...")
        jointVectorIterations = np.concatenate([jointVectorIterations,jointVector])
    print(jointVectorIterations)
    print("SE(3) end-effector config :  \n", SE3)
    print("error twist V_b : ", errorTwist)
    print("Angular error maginitude ||omega_b|| : ", angularError)
    print("Linear error magnitude ||v_b|| : ", linearError)
    print("\n")

    return jointVectorIterations

"""
RUNNING THE PROGRAM
Dimesions: given in chapter 4.1.2
"""
L1 = 425/1000 #mm
L2 = 392/1000
H1 = 89/1000
H2 = 95/1000
W1 = 109/1000
W2 = 82/1000

Blist = np.array([[0,1,0,W1+W2,0,L1 + L2],[0,0,1,H2,-L1-L2,0],[0,0,1,H2,-L2,0],[0,0,1,H2,0,0],[0,-1,0,-W2,0,0],[0,0,1,0,0,0]]).T
M = np.array([[-1,0,0,L1 + L2],[0,0,1,W1 + W2],[0,1,0,H1 - H2],[0,0,0,1]])
T = np.array([[0,1,0,-0.5],[0,0,-1,0.1],[-1,0,0,0.1],[0,0,0,1]])
thetalist0 = np.array([6.014,-2.293,-1.761,0.565,3.422,1.296]) #Initial Guess
eomg = 0.001
ev = 0.0001

IKinBodyIterates(Blist,M,T,thetalist0,eomg,ev)
