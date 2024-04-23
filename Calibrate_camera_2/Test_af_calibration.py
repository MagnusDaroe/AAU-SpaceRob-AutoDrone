"""
NOTE:
    - The code is written in Python 3.9.12.
    - Using OpenCV contrib-python 4.6.0.66
"""

import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

# Load the camera calibration parameters
with np.load("Calibrate_camera_2\calib_2.npz") as X:
    ret,mtx, dist = [X[i] for i in ("ret","mtx", "dist")]

datadir = "Calibrate_camera_2\Img_for_calibration_2\\"
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".jpg") ])
order = np.argsort([int(p.split("\\")[-1].split(".")[0]) for p in images])
images = images[order]

img_no=7
frame = cv2.imread(images[img_no])
frame = cv2.undistort(src = frame, cameraMatrix = mtx, distCoeffs = dist)
plt.figure()
plt.imshow(frame, interpolation = "nearest")
plt.show()

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                      parameters=parameters)
# SUB PIXEL DETECTION
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
for corner in corners:
    cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)


plt.figure()
plt.imshow(frame_markers, interpolation = "nearest")
plt.show()

size_of_marker =  0.0466 # side lenght of the marker in meter
rvecs,tvecs,_objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)

length_of_axis = 0.1
imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

for i in range(len(tvecs)):
    #imaxis = aruco.drawAxis(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)
    imaxis=cv2.drawFrameAxes(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)


plt.figure()
plt.imshow(imaxis)
plt.grid()
plt.show()

data = pd.DataFrame(data = tvecs.reshape(len(tvecs),3), columns = ["tx", "ty", "tz"],
                    index = ids.flatten())
data.index.name = "marker"
data.sort_index(inplace= True)

print(data)

datar = pd.DataFrame(data = tvecs.reshape(len(rvecs),3), columns = ["rx", "ry", "rz"],
                    index = ids.flatten())
datar.index.name = "marker"
datar.sort_index(inplace= True)
print(np.degrees(datar))

v = data.loc[3:6].values
print(((v[1:] - v[:-1])**2).sum(axis = 1)**.5)

print(cv2.Rodrigues(rvecs[0], np.zeros((3,3))))

fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
ax = fig.add_subplot(1,2,1)
ax.set_aspect("equal")
plt.plot(data.tx, data.ty, "or-")
plt.grid()
ax = fig.add_subplot(1,2,2)
plt.imshow(imaxis, origin = "upper")
plt.plot(np.array(corners)[:, 0, 0,0], np.array(corners)[:, 0, 0,1], "or")
plt.show()

