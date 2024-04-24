import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

"""
NOTE:
    - The code is written in Python 3.9.12.
    - Using OpenCV contrib-python 4.6.0.66
"""

workdir = "Calibrate_camera_2"
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
#board = aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)
board = aruco.CharucoBoard_create(7, 5, 0.0582, .0466, aruco_dict) #58.2, 46.6

imboard = board.draw((2000, 2000))

##cv2.imwrite("{}\chessboard.tiff".format(workdir),imboard)
#cv2.imwrite("{}\chessboard.png".format(workdir),imboard)
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
plt.imshow(imboard, cmap = mpl.cm.gray, interpolation = "nearest")
ax.axis("off")
plt.show()



datadir = "Calibrate_camera_2\Img_for_calibration_2\\"
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".jpg") ])
order = np.argsort([int(p.split("\\")[-1].split(".")[0]) for p in images])
images = images[order]
print(images)
images

img=cv2.imread("Calibrate_camera_2\Img_for_calibration_2\\2.jpg")
cv2.imshow("img",img)
cv2.waitKey(0)
cv2.destroyAllWindows()

im = PIL.Image.open(images[0])
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
plt.imshow(im)
#ax.axis('off')
plt.show()

def read_chessboards(images):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize = (3,3),
                                 zeroZone = (-1,-1),
                                 criteria = criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator+=1

    imsize = gray.shape
    return allCorners,allIds,imsize


allCorners,allIds,imsize=read_chessboards(images)

def calibrate_camera(allCorners,allIds,imsize):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
                                 [    0., 1000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    #flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors

#time 
ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners,allIds,imsize)
print("ret:\n",ret)
print("\nmtx:\n",mtx)
print("\ndist:\n",dist)
#print("\nrvecs:\n",rvecs)
#print("\ntvecs:\n",tvecs)

#save the camera matrix and distortion coefficients to a file
np.savez("Calibrate_camera_2\calib_5.npz", ret=ret, mtx=mtx, dist=dist)


i=7 # select image id
plt.figure()
frame = cv2.imread(images[i])
img_undist = cv2.undistort(frame,mtx,dist,None)
plt.subplot(1,2,1)
plt.imshow(frame)
plt.title("Raw image")
plt.axis("off")
plt.subplot(1,2,2)
plt.imshow(img_undist)
plt.title("Corrected image")
plt.axis("off")
plt.show()

"""
Version 1: 
with --> aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)

ret:
 0.5217429294478918

mtx:
 [[286.89037106   0.         422.19025285]
 [  0.         286.89037106 400.53361354]
 [  0.           0.           1.        ]]

dist:
 [[ 3.23682127e+00]
 [ 1.08655572e+00]
 [-8.98770664e-05]
 [ 1.02127945e-04]
 [ 2.63414184e-02]
 [ 3.58479074e+00]
 [ 2.01296405e+00]
 [ 1.91389997e-01]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]]
"""
"""
Version 2:
ret:
 0.5211782577035629

mtx:
 [[286.88540368   0.         422.20048622]
 [  0.         286.88540368 400.53275847]
 [  0.           0.           1.        ]]

dist:
 [[ 3.20245253e+00]
 [ 1.07256948e+00]
 [-8.98646466e-05]
 [ 1.01222477e-04]
 [ 2.60095104e-02]
 [ 3.55015891e+00]
 [ 1.98817304e+00]
 [ 1.88916559e-01]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]]
"""
"""
Version 3:
ret:
 0.5211782577035629

mtx:
 [[286.88540368   0.         422.20048622]
 [  0.         286.88540368 400.53275847]
 [  0.           0.           1.        ]]

dist:
 [[ 3.20245253e+00]
 [ 1.07256948e+00]
 [-8.98646466e-05]
 [ 1.01222477e-04]
 [ 2.60095104e-02]
 [ 3.55015891e+00]
 [ 1.98817304e+00]
 [ 1.88916559e-01]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]]
"""
"""
Version 4:
ret:
 0.48922053203059485

mtx:
 [[286.41104486   0.         422.10588356]
 [  0.         286.41104486 401.12452825]
 [  0.           0.           1.        ]]

dist:
 [[ 2.87824329e+00]
 [ 8.91290335e-01]
 [-1.51634209e-04]
 [ 1.19020932e-04]
 [ 2.06907816e-02]
 [ 3.22667840e+00]
 [ 1.69796937e+00]
 [ 1.52717654e-01]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]]
"""