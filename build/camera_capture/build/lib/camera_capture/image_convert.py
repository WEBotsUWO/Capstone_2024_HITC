import os
import cv2
from cv2 import aruco, imshow
import numpy as np
import matplotlib.pyplot as plt


def updateMaze(savePath = '/home/chris/capstone/hitc_ws/src/camera_capture/camera_capture'):
    mtx = np.genfromtxt("/home/chris/capstone/hitc_ws/src/camera_capture/camera_capture/camerMatrix.txt", dtype=float, encoding=None, delimiter=',')
    dist = np.genfromtxt("/home/chris/capstone/hitc_ws/src/camera_capture/camera_capture/cameraDistortion.txt", dtype=float, encoding=None, delimiter=',')

    fontSize = 24

    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    rgbMaze = cv2.undistort(frame, mtx, dist)
    # rgbMaze = cv2.imread("C:/Users/niloc/OneDrive/Desktop/Capstone/Code/maze.jpg")

    # Get image dimensions
    rows, columns, numberOfColorChannels = rgbMaze.shape

    # # Display the original color image
    plt.subplot(1, 2, 1)
    plt.imshow(cv2.cvtColor(rgbMaze, cv2.COLOR_BGR2RGB))
    plt.title('Original Color Image', fontsize=fontSize)

    # # Extract color channels
    redChannel = rgbMaze[:, :, 2]
    greenChannel = rgbMaze[:, :, 1]
    blueChannel = rgbMaze[:, :, 0]

    # # Create a grayscale image
    grayMaze = rgbMaze.copy()

    # # Process each pixel in the image
    for i in range(rows):
        for j in range(columns):
            if redChannel[i, j] < 140 and greenChannel[i, j] < 140 and blueChannel[i, j] < 140:
                grayMaze[i, j, :] = 0 
            elif redChannel[i, j] <= 70 and greenChannel[i, j] <= 130 and blueChannel[i, j] >= 170: 
                grayMaze[i, j, :] = 0 
            else:
                grayMaze[i, j, :] = 255

    # Display the converted image
    plt.subplot(1, 2, 2)
    plt.imshow(cv2.cvtColor(grayMaze, cv2.COLOR_BGR2RGB))
    plt.title('Converted Image', fontsize=fontSize)

    path = savePath
    cv2.imwrite(os.path.join(path, 'processed_workspace.png'), grayMaze)

    # plt.show()