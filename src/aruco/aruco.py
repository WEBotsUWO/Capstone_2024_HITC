import numpy as np
import cv2
from cv2 import aruco

# Load your camera calibration parameters (replace these values with your calibration results)
mtx = np.genfromtxt("cameraMatrix.txt", dtype=float, encoding=None, delimiter=',')
dist = np.genfromtxt("cameraDistortion.txt", dtype=float, encoding=None, delimiter=',')

cap = cv2.VideoCapture(1)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    parameters = aruco.DetectorParameters()
    corners, ids, rejectedImgPoints = aruco.ArucoDetector(aruco_dict, parameters).detectMarkers(frame)
    marker_size = 5 #Update to the size of the marker
    if ids is not None:
        
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)

        for i in range(len(ids)):
            distance = np.linalg.norm(tvecs[i])
            cv2.putText(frame, f"Distance: {distance:.2f} units", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            print(distance)

    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    cv2.imshow('frame_marker', frame_markers)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()