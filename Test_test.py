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


T_FC_backside=np.array([[1,0,0,153.223],[0,1,0,0],[0,0,1,86.6070],[0,0,0,1]]) #mm

# Transformation from backside center of T265 to cam frame center in T265:
T_backside_center=np.array([[1,0,0,1.54],[0,1,0,-9.10],[0,0,1,05.75],[0,0,0,1]]) #mm

# Transformation from pose frame to the FC:
T_pose_FC=inv(T_FC_backside@T_backside_center)

print("T_pose_FC: \n",T_pose_FC)

T_FC_pose=T_FC_backside@T_backside_center
print("T_FC_pose: \n",T_FC_pose)

t_vec_flat_meter=np.array([0.0862,0.0645,0.0])

P_Cam_marker=np.array([[t_vec_flat_meter[0]*1000],[t_vec_flat_meter[1]*1000],[t_vec_flat_meter[2]*1000]])
theta=np.deg2rad(75)
rotz_75=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
P_CamFlat_marker=rotz_75@P_Cam_marker

print("P_CamFlat_marker: ",P_CamFlat_marker)
