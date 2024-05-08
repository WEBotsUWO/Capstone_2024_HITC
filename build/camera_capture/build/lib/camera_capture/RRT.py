import numpy as np
import matplotlib.pyplot as plt
import random
from camera_capture.treeNode import treeNode
#settings of python plots
from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 22

# #treeNode class

# class treeNode():
#     def __init__(self, locationX, locationY):
#         self.locationX = locationX
#         self.locationY = locationY
#         self.value = [locationX, locationY] # we need to store their position in space to check for collisions.
#         self.children = [] # parent can have many children
#         self.parent = None # each node can only have one parent

class RRTAlgorithm():
    '''
    This class contains helper methods that allow us to implement the RRT algorithm  easier

    To perform RRT we need to be able to :
    - sample a random point in the c-space.
    - Need to be able to move a certain step size from your current position in the direction of your sampled point
    - You need to determine whether you have hit an obstacle or not
    - you need to determine the direction the sampled point is relative to the nearest node.
    - you need to determine what the nearest node is.
    - 
    '''
    def __init__(self, start, goal, numIterations,grid, stepSize):
        

        self.randomTree = treeNode(start[0],start[1]) #root of tree
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.iterations = min(numIterations,10000)
        self.grid = grid
        self.rho = stepSize
        self.path_distance = 0
        self.nearestDist = 150
        self.numWayPoints = 0
        self.Waypoints = []

    #Add the point to the nearest node and add goal when reached

    def addChild(self, locationX, locationY):
       if(locationX == self.goal.locationX):
           self.nearestNode.children.append(self.goal)
           self.goal.parent = self.nearestNode
       else:
           tempNode = treeNode(locationX,locationY)
           self.nearestNode.children.append(tempNode)
           tempNode.parent = self.nearestNode
    #sample a random point within grid limits
    def sampleAPoint(self):
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        point = np.array([x,y])
        return point
    
    #steer a distance stepsize form start to end location
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])

        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1]-1
        if point[1] >= self.grid.shape[0]:
            point[1] = self.grid.shape[0]-1           
        return point

    #check if obstacle lies between the start node and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0, 0])

        for i in range(self.rho):
            testPoint[0] = locationStart.locationX + i*u_hat[0]
            testPoint[1] = locationStart.locationY + i*u_hat[1]
            #check if testPoint Lies within obstacle
            # print( int(testPoint))

            testPoint[0] = min(testPoint[0], self.grid.shape[1]-1)
            testPoint[1] = min(testPoint[1], self.grid.shape[0]-1)
            if self.grid[testPoint[1], testPoint[0]] == 1:
                return True
        return False
    
    #find unit vector between a node and an end point which form a vector
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    

    #find the nearest node from a given unconnected point (Euclidean distance)
    def findNearest(self,root,point):
        if not root:
            return
        
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        
        #recursively call by iterating through the children
        for child in root.children:
            self.findNearest(child, point)
        
    #find euclidean distance betweeen a node and an XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist
    
    #check if the goal has been reached within step size
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        
    # reset nearestNode and nearestDistance
    def resetNearestValues(self):
        self.nearesNode = None
        self.nearestDist = 10000

    #trace the path from goal to start
    def retraceRRTPath(self, goal):
       if goal.locationX == self.randomTree.locationX:
           return
       self.numWayPoints += 1
       #insert currentPoint to the Waypoints array from the beginning
       currentPoint = np.array([goal.locationX, goal.locationY])
       self.Waypoints.insert(0,currentPoint)
       self.path_distance += self.rho
       self.retraceRRTPath(goal.parent)


def solveMaze(startPoint = [605,84], goalPoint = [59,418], workspace_path = '/home/chris/capstone/hitc_ws/src/camera_capture/camera_capture/processed_workspace.npy', iterations = 1000, stepLength = 50):
        #end of class method definitions -------------------------------

        #load the frid, set start and gaol <x,y> positions, number of iterations, step size
        # 0,0 is top left corner
        grid = np.load(workspace_path)
        # start = np.array([1228.864, 107.367])
        # goal = np.array([311.630, 800.650])

        start = np.array(startPoint)
        goal = np.array(goalPoint)


        numIterations = iterations
        stepSize = stepLength
        goalRegion = plt.Circle((goal[0],goal[1]), stepSize, color='b', fill=False)

        fig = plt.figure("RRT Algorithm")
        plt.imshow(grid, cmap='binary')
        plt.plot(start[0],start[1],'ro')
        plt.plot(goal[0],goal[1],'bo')
        ax = fig.gca()
        ax.add_patch(goalRegion)
        plt.xlabel('X-axis $(m)$')
        plt.ylabel('Y-axis $(m)$')



        rrt = RRTAlgorithm(start,goal,numIterations,grid,stepSize)

        print("Opening graph now ...")
        for i in range(rrt.iterations):
            #Reset nearest values
            rrt.resetNearestValues()
            print("Iteration: ", i)
            #algorithm begins here
            point = rrt.sampleAPoint()
            rrt.findNearest(rrt.randomTree, point)
            new = rrt.steerToPoint(rrt.nearestNode, point)
            bool = rrt.isInObstacle(rrt.nearestNode, new)
            if(bool == False):
                rrt.addChild(new[0],new[1])
                plt.pause(0.10)
                plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]],'go',linestyle="--")
                #if goal found, append to path
                if (rrt.goalFound(new)):
                    rrt.addChild(goal[0],goal[1])
                    print("Goal found!")
                    break


        #trace back the path returned, and add start to waypoints

        rrt.retraceRRTPath(rrt.goal)
        rrt.Waypoints.insert(0,start)
        print("Num of waypoints: ", rrt.numWayPoints)
        print("Path Distance(m): ", rrt.path_distance)
        print("Waypoints: ", rrt.Waypoints)

        #plot waypoints

        for i in range(len(rrt.Waypoints) - 1):
            plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro', linestyle="--")
            plt.pause(0.10)


        plt.savefig("currentMazeSolution.png")
        # plt.show()
        # plt.pause(0.10)
    


 