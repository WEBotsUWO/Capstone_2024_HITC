import os
import cv2
from cv2 import aruco, imshow
import numpy as np
import matplotlib.pyplot as plt

mtx = np.genfromtxt("/home/chris/capstone/hitc_ws/src/camera_capture/camera_capture/camerMatrix.txt", dtype=float, encoding=None, delimiter=',')
dist = np.genfromtxt("/home/chris/capstone/hitc_ws/src/camera_capture/camera_capture/cameraDistortion.txt", dtype=float, encoding=None, delimiter=',')

fontSize = 24

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
rgbMaze = cv2.undistort(frame, mtx, dist)
# Read the image
# rgbMaze = cv2.imread("C:/Users/niloc/OneDrive/Desktop/Capstone/test_maze.png")

# Get image dimensions
rows, columns, numberOfColorChannels = rgbMaze.shape

# # Display the original color image
plt.subplot(1, 3, 1)
plt.imshow(cv2.cvtColor(rgbMaze, cv2.COLOR_BGR2RGB))
plt.title('Original Color Image', fontsize=fontSize)

# # Enlarge figure to full screen
plt.gcf().set_size_inches(18, 6)

# # Extract color channels
redChannel = rgbMaze[:, :, 2]
greenChannel = rgbMaze[:, :, 1]
blueChannel = rgbMaze[:, :, 0]

# # Create a grayscale image
grayMaze = rgbMaze.copy()

# # Process each pixel in the image
for i in range(rows):
    for j in range(columns):
        if redChannel[i, j] < 100 and greenChannel[i, j] < 100 and blueChannel[i, j] < 100:
            grayMaze[i, j, :] = 0 
        elif redChannel[i, j] >= 150 and greenChannel[i, j] <= 100 and blueChannel[i, j] <= 100:
            grayMaze[i, j, 2] = 255
            grayMaze[i, j, 1] = 0 
            grayMaze[i, j, 0] = 0
        elif redChannel[i, j] <= 160 and greenChannel[i, j] >= 200 and blueChannel[i, j] <= 160:
            grayMaze[i, j, 2] = 0 
            grayMaze[i, j, 1] = 255 
            grayMaze[i, j, 0] = 0 
        elif redChannel[i, j] <= 70 and greenChannel[i, j] <= 130 and blueChannel[i, j] >= 170:
            grayMaze[i, j, 2] = 0 
            grayMaze[i, j, 1] = 0 
            grayMaze[i, j, 0] = 255 
        else:
            grayMaze[i, j, :] = 255

# Display the converted image
plt.subplot(1, 3, 2)
plt.imshow(cv2.cvtColor(grayMaze, cv2.COLOR_BGR2RGB))
plt.title('Converted Image', fontsize=fontSize)
# plt.show()

# Extract blue channel
blueChannel = grayMaze[:, :, 0]
greenChannel = grayMaze[:, :, 1]
redChannel = grayMaze[:,:, 2]

# Create a mask for the background
mask = blueChannel > 200
mask2 = redChannel > 200
# Mask out the background
maskedRgbImage = np.multiply(grayMaze, np.stack([~mask, ~mask, ~mask], axis=-1))
masked2 = np.multiply(grayMaze, np.stack([~mask2, ~mask2, ~mask2], axis=-1))
# Initialize redDot and greenDot
redDot = maskedRgbImage.copy()
greenDot = maskedRgbImage.copy()
blueDot = masked2.copy()

# Extract color channels
redChannel = maskedRgbImage[:, :, 2]
greenChannel = maskedRgbImage[:, :, 1]
blueChannel = masked2[:, :, 0]

# Binarize red and green channels
bwr = cv2.threshold(redChannel, 0, 255, cv2.THRESH_BINARY)[1]
bwg = cv2.threshold(greenChannel, 0, 255, cv2.THRESH_BINARY)[1]
bwb = cv2.threshold(blueChannel, 0, 255, cv2.THRESH_BINARY)[1]

# Process each pixel in the image
for i in range(rows):
    for j in range(columns):
        if bwr[i, j] == 255 and bwg[i, j] == 0 and bwb[i,j] == 0:
            greenDot[i, j, :] = 0
            blueDot[i, j, :] = 0
            redDot[i, j, :] = 255
        elif bwr[i, j] == 0 and bwg[i, j] == 255 and bwb[i,j] == 0:
            greenDot[i, j, :] = 255
            blueDot[i, j, :] = 0
            redDot[i, j, :] = 0
        elif bwr[i, j] == 0 and bwg[i, j] == 0 and bwb[i,j] == 255:
            greenDot[i, j, :] = 0
            blueDot[i, j, :] = 255
            redDot[i, j, :] = 0
        else:
            redDot[i, j, :] = 0
            greenDot[i, j, :] = 0
            blueDot[i, j , :] = 0

# imshow("Red", redDot)
# imshow("Green", greenDot)
# imshow("Blue", blueDot)

# Binarize redDot and greenDot channels
redBW = cv2.threshold(redDot[:, :, 2], 0, 255, cv2.THRESH_BINARY)[1]
greenBW = cv2.threshold(greenDot[:, :, 1], 0, 255, cv2.THRESH_BINARY)[1]
blueBW = cv2.threshold(blueDot[:, :, 1], 0, 255, cv2.THRESH_BINARY)[1]

# Combine red and green binary images
combineBW = cv2.addWeighted(redBW, 1, greenBW, 1, 0)

imshow("Red", redBW)
imshow("Green", greenBW)
imshow("Blue", blueBW)

# Find centroids
redCenter = cv2.findContours(redBW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
greenCenter = cv2.findContours(greenBW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
blueCenter = cv2.findContours(blueBW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]

# imshow("Red", redBW)
# imshow("Green", greenBW)
# imshow("Blue", blueBW)


print("Red", redCenter)
print("Green", greenCenter)
# print("Blue", blueCenter)

if len(redCenter) > 0:
    redMoments = cv2.moments(redCenter)
    # print(redMoments)
    redX = int(redMoments["m10"] / redMoments["m00"])
    redY = int(redMoments["m01"] / redMoments["m00"])
    redCoords = [redX, redY]
    print('Red: ', redCoords)

if len(greenCenter) > 0:
    greenMoments = cv2.moments(greenCenter)
    greenX = int(greenMoments["m10"] / greenMoments["m00"])
    greenY = int(greenMoments["m01"] / greenMoments["m00"])
    greenCoords = [greenX, greenY]
    print("Green: ", greenCoords)

if len(blueCenter) > 0:
    blueMoments = cv2.moments(blueCenter)
    blueX = int(blueMoments["m10"] / blueMoments["m00"])
    blueY = int(blueMoments["m01"] / blueMoments["m00"])
    blueCoords = [blueX, blueY]
    print("Blue: ", blueCoords)

tMaze = grayMaze.copy()

# Process each pixel in the image based on combineBW
for i in range(rows):
    for j in range(columns):
        if combineBW[i, j] == 255:
            tMaze[i, j, :] = 255
        if blueBW[i, j] == 255:
            tMaze[i,j] = 0

# plt.imshow(cv2.cvtColor(tMaze, cv2.COLOR_BGR2RGB))
# plt.show()

path = '/home/chris/capstone/hitc_ws/src/camera_capture/camera_capture'

# cv2.imwrite(os.path.join(path, 'converted_image.jpg'), tMaze)

# # Display the final white dot image
plt.subplot(1, 3, 3)
plt.imshow(cv2.cvtColor(tMaze, cv2.COLOR_BGR2RGB))
plt.title('White Dot Image', fontsize=fontSize)
plt.show()



# import os
# import cv2
# from cv2 import aruco, imshow
# import numpy as np
# import matplotlib.pyplot as plt

# # def get_maze_details():
# mtx = np.genfromtxt("cameraMatrix.txt", dtype=float, encoding=None, delimiter=',')
# dist = np.genfromtxt("cameraDistortion.txt", dtype=float, encoding=None, delimiter=',')

# fontSize = 24

# cap = cv2.VideoCapture(0)
# ret, frame = cap.read()
# rgbMaze = cv2.undistort(frame, mtx, dist)

# path = '/home/chris/Downloads/aruco_detection'
# cv2.imwrite(os.path.join(path, 'cameraImage.jpg'), rgbMaze)

# # Read the image
# # rgbMaze = cv2.imread("C:/Users/niloc/OneDrive/Desktop/Capstone/test_maze.png")

# # Get image dimensions
# rows, columns, numberOfColorChannels = rgbMaze.shape

# # # Display the original color image
# plt.subplot(1, 3, 1)
# plt.imshow(cv2.cvtColor(rgbMaze, cv2.COLOR_BGR2RGB))
# plt.title('Original Color Image', fontsize=fontSize)

# # # Enlarge figure to full screen
# plt.gcf().set_size_inches(18, 6)

# # # Extract color channels
# redChannel = rgbMaze[:, :, 2]
# greenChannel = rgbMaze[:, :, 1]
# blueChannel = rgbMaze[:, :, 0]

# # # Create a grayscale image
# grayMaze = rgbMaze.copy()

# # # Process each pixel in the image
# for i in range(rows):
#     for j in range(columns):
#         if redChannel[i, j] < 100 and greenChannel[i, j] < 100 and blueChannel[i, j] < 100:
#             grayMaze[i, j, :] = 0 
#         elif redChannel[i, j] >= 150 and greenChannel[i, j] <= 100 and blueChannel[i, j] <= 100:
#             grayMaze[i, j, 2] = 255
#             grayMaze[i, j, 1] = 0 
#             grayMaze[i, j, 0] = 0
#         elif redChannel[i, j] <= 160 and greenChannel[i, j] >= 200 and blueChannel[i, j] <= 160:
#             grayMaze[i, j, 2] = 0 
#             grayMaze[i, j, 1] = 255 
#             grayMaze[i, j, 0] = 0 
#         elif redChannel[i, j] <= 70 and greenChannel[i, j] <= 130 and blueChannel[i, j] >= 170:
#             grayMaze[i, j, 2] = 0 
#             grayMaze[i, j, 1] = 0 
#             grayMaze[i, j, 0] = 255 
#         else:
#             grayMaze[i, j, :] = 255

# # Display the converted image
# plt.subplot(1, 3, 2)
# plt.imshow(cv2.cvtColor(grayMaze, cv2.COLOR_BGR2RGB))
# plt.title('Converted Image', fontsize=fontSize)
# # plt.show()

# # Extract blue channel
# blueChannel = grayMaze[:, :, 0]
# greenChannel = grayMaze[:, :, 1]
# redChannel = grayMaze[:,:, 2]

# # Create a mask for the background
# mask = blueChannel > 200
# mask2 = redChannel > 200
# # Mask out the background
# maskedRgbImage = np.multiply(grayMaze, np.stack([~mask, ~mask, ~mask], axis=-1))
# masked2 = np.multiply(grayMaze, np.stack([~mask2, ~mask2, ~mask2], axis=-1))
# # Initialize redDot and greenDot
# redDot = maskedRgbImage.copy()
# greenDot = maskedRgbImage.copy()
# blueDot = masked2.copy()

# # Extract color channels
# redChannel = maskedRgbImage[:, :, 2]
# greenChannel = maskedRgbImage[:, :, 1]
# blueChannel = masked2[:, :, 0]

# # Binarize red and green channels
# bwr = cv2.threshold(redChannel, 0, 255, cv2.THRESH_BINARY)[1]
# bwg = cv2.threshold(greenChannel, 0, 255, cv2.THRESH_BINARY)[1]
# bwb = cv2.threshold(blueChannel, 0, 255, cv2.THRESH_BINARY)[1]

# # Process each pixel in the image
# for i in range(rows):
#     for j in range(columns):
#         if bwr[i, j] == 255 and bwg[i, j] == 0 and bwb[i,j] == 0:
#             greenDot[i, j, :] = 0
#             blueDot[i, j, :] = 0
#             redDot[i, j, :] = 255
#         elif bwr[i, j] == 0 and bwg[i, j] == 255 and bwb[i,j] == 0:
#             greenDot[i, j, :] = 255
#             blueDot[i, j, :] = 0
#             redDot[i, j, :] = 0
#         elif bwr[i, j] == 0 and bwg[i, j] == 0 and bwb[i,j] == 255:
#             greenDot[i, j, :] = 0
#             blueDot[i, j, :] = 255
#             redDot[i, j, :] = 0
#         else:
#             redDot[i, j, :] = 0
#             greenDot[i, j, :] = 0
#             blueDot[i, j , :] = 0

# # imshow("Red", redDot)
# # imshow("Green", greenDot)
# # imshow("Blue", blueDot)

# # Binarize redDot and greenDot channels
# redBW = cv2.threshold(redDot[:, :, 2], 0, 255, cv2.THRESH_BINARY)[1]
# greenBW = cv2.threshold(greenDot[:, :, 1], 0, 255, cv2.THRESH_BINARY)[1]
# blueBW = cv2.threshold(blueDot[:, :, 1], 0, 255, cv2.THRESH_BINARY)[1]

# # Combine red and green binary images
# combineBW = cv2.addWeighted(redBW, 1, greenBW, 1, 0)

# # imshow("Red", redBW)
# # imshow("Green", greenBW)
# # imshow("Blue", blueBW)

# # Find centroids
# redCenter = cv2.findContours(redBW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
# greenCenter = cv2.findContours(greenBW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
# blueCenter = cv2.findContours(blueBW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]

# # imshow("Red", redBW)
# # imshow("Green", greenBW)
# # imshow("Blue", blueBW)


# print("Red", redCenter)
# print("Green", greenCenter)
# print("Blue", blueCenter)

# # if len(redCenter) > 0:
# #     redMoments = cv2.moments(redCenter)
# #     # print(redMoments)
# #     redX = int(redMoments["m10"] / redMoments["m00"])
# #     redY = int(redMoments["m01"] / redMoments["m00"])
# #     redCoords = [redX, redY]
# #     print('Red: ', redCoords)

# # if len(greenCenter) > 0:
# #     greenMoments = cv2.moments(greenCenter)
# #     greenX = int(greenMoments["m10"] / greenMoments["m00"])
# #     greenY = int(greenMoments["m01"] / greenMoments["m00"])
# #     greenCoords = [greenX, greenY]
# #     print("Green: ", greenCoords)

# # if len(blueCenter) > 0:
# #     blueMoments = cv2.moments(blueCenter)
# #     blueX = int(blueMoments["m10"] / blueMoments["m00"])
# #     blueY = int(blueMoments["m01"] / blueMoments["m00"])
# #     blueCoords = [blueX, blueY]
# #     print("Blue: ", blueCoords)

# # tMaze = grayMaze.copy()

# # # Process each pixel in the image based on combineBW
# # for i in range(rows):
# #     for j in range(columns):
# #         if combineBW[i, j] == 255:
# #             tMaze[i, j, :] = 255
# #         if blueBW[i, j] == 255:
# #             tMaze[i,j] = 0

# # # plt.imshow(cv2.cvtColor(tMaze, cv2.COLOR_BGR2RGB))
# # # plt.show()

# # path = '/home/chris/capstone/hitc_ws/src/trajectory_generator/trajectory_generatorstate_publisher_py'

# # cv2.imwrite(os.path.join(path, 'converted_image.jpg'), tMaze)  # send grayscale image to state_publisher_py.

# # # Display the final white dot image
# # plt.subplot(1, 3, 3)
# # plt.imshow(cv2.cvtColor(tMaze, cv2.COLOR_BGR2RGB))
# # plt.title('White Dot Image', fontsize=fontSize)
# # plt.show()

# # return [greenCenter, redCenter]

