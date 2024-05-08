import numpy as np
import matplotlib.pyplot as plt
import random
from camera_capture.RRT import RRTAlgorithm

#settings of python plots
from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 22

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

        plt.show()
