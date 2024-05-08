import numpy as np
import matplotlib.pyplot as plt
import random
from trajectory_generator.RRTAlgorithm import RRTAlgorithm
#settings of python plots
from PIL import Image, ImageOps
from matplotlib.pyplot import rcParams

class student_trajectory:
    def run_algorithm(self,start, goal,processed_image_path = "/home/chris/capstone/hitc_ws/src/trajectory_generator/trajectory_generator/Converted_Maze.png"):  

        np.set_printoptions(precision=3, suppress=True)
        rcParams['font.family'] = 'sans-serif'
        rcParams['font.sans-serif'] = ['Tahoma']
        plt.rcParams['font.size'] = 22
        #load the frid, set start and gaol <x,y> positions, number of iterations, step size
        # 0,0 is top left corner

        image = Image.open(processed_image_path)
        np_img = np.array(image)
        np.save('/home/chris/capstone/hitc_ws/src/trajectory_generator/trajectory_generator/Converted_Maze.npy', np_img)

        grid = np.load('/home/chris/capstone/hitc_ws/src/trajectory_generator/trajectory_generator/Converted_Maze.npy')
        # start = np.array([1228.864, 107.367])
        # goal = np.array([311.630, 800.650])


        numIterations = 1000
        stepSize = 100
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
            # print("Iteration: ", i)
            #algorithm begins here
            point = rrt.sampleAPoint()
            rrt.findNearest(rrt.randomTree, point)
            new = rrt.steerToPoint(rrt.nearestNode, point)
            bool = rrt.isInObstacle(rrt.nearestNode, new)
            if(bool == False):
                rrt.addChild(new[0],new[1])
                # plt.pause(0.10) # pause to plot points in real time.
                # plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]],'go',linestyle="--")
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
        print("Waypoints: ", rrt.Waypoints[0])
        print("Waypoints size: ", len(rrt.Waypoints))
    
        #plot waypoints

        for i in range(len(rrt.Waypoints) - 1):
            plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro', linestyle="--")
            plt.pause(0.10)

        plt.show()

        return list(rrt.Waypoints) # already a list() object
    
    # def convertImgToNpy(self,image_path):
    #     image = Image.open(image_path)
    #     np_img = np.array(image)
    #     np.save('/home/chris/capstone/hitc_ws/src/trajectory_generator/trajectory_generator/Converted_Maze.npy', np_img)
        