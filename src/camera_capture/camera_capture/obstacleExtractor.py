from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt


def locate_obstacles():
    img = Image.open('/home/chris/capstone/hitc_ws/src/camera_capture/camera_capture/processed_workspace.png')
    img = ImageOps.grayscale(img)

    np_img = np.array(img)
    np_img = ~np_img
    np_img[np_img > 0] = 1
    plt.set_cmap('binary')
    plt.imshow(np_img)

    #Save Image
    np.save('/home/chris/capstone/hitc_ws/src/camera_capture/camera_capture/processed_workspace.npy', np_img)

    # grid = np.load('Converted_Maze.npy')
    # plt.imshow(grid, cmap = "binary")
    # plt.tight_layout()
    # plt.show()
