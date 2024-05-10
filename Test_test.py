#Prrint version of opencv
import cv2
print("Opencv version: ",cv2.__version__)
#Print version of numpy
import numpy as np
print("Numpy version: ",np.__version__)
from numpy.linalg import inv
__CALIBRATION_FILE = "_Calibrate_camera_2\calib_new_1.npz"

with np.load(__CALIBRATION_FILE) as X:
    ret,mtx, dist = [X[i] for i in ("ret","mtx", "dist")]

print("ret: \n",ret)
print("\nmtx: \n",mtx)
print("\ndist: \n",dist)

MT=np.array( [[289.17101938,0.,426.23687843],[0.,289.14205298,401.22256516],[0.,0.,1.]])
DT=np.array([[-0.22814816,0.04330513,-0.00027584,0.00057192,-0.00322855]])
RT=18
print(MT)
print(DT)
print(RT)


R=np.array([[1,0,0],[1,0,0],[0,0,1]])
#finde extrinsic roll pitch yaw
roll = np.arctan2(R[2,1], R[2,2])
pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
yaw = np.arctan2(R[1,0], R[0,0])
print("roll: ",roll)
print("pitch: ",pitch)
print("yaw: ",yaw)

print("rad2deg(2.881954)=",np.rad2deg(2.881954))

R=np.array([[0,-1,0],[0,0,1],[-1,0,0]])

#multiply vector with matrix

print("6,-3,5: ",np.array([[0,0,-1],[-1,0,0],[0,1,0]])@np.array([[3],[5],[-6]]))
hej=np.array([[0,0,-1],[-1,0,0],[0,1,0]])
print(hej)


angle=-75
T_ref_cam=np.array([[np.cos(np.deg2rad(angle)),0,np.sin(np.deg2rad(angle)),0],
                            [0,1,0,0],
                            [-np.sin(np.deg2rad(angle)),0,np.cos(np.deg2rad(angle)),0],
                            [0,0,0,1]])

print("T_ref_cam -75: \n",T_ref_cam)
print("T_ref_cam inv(-75): \n",inv(T_ref_cam))
angle=+180
T_ref_cam=np.array([[np.cos(np.deg2rad(angle)),0,np.sin(np.deg2rad(angle)),0],
                            [0,1,0,0],
                            [-np.sin(np.deg2rad(angle)),0,np.cos(np.deg2rad(angle)),0],
                            [0,0,0,1]])

print("T_ref_cam +75: \n",T_ref_cam)