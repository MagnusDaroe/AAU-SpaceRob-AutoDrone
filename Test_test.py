#Prrint version of opencv
import cv2
print("Opencv version: ",cv2.__version__)
#Print version of numpy
import numpy as np
print("Numpy version: ",np.__version__)

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