import numpy as np
import cv2
import matplotlib.pyplot as plt
dist=np.array([[-0.22814816,0.04330513,-0.00027584,0.00057192,-0.00322855]])
mtx=np.array( [[289.17101938,0.,426.23687843],[0.,289.14205298,401.22256516],[0.,0.,1.]])
#Undistort 7.jpg and show the undistorted image beside the original/raw image with the use of dist and mtx

frame = cv2.imread("TEST_w_Vicon\\7.jpg")
img_undist = cv2.undistort(src = frame, cameraMatrix = mtx, distCoeffs = dist)
plt.subplot(1,2,1)
plt.imshow(frame)
plt.title("Raw image")
plt.axis("off")
plt.subplot(1,2,2)
plt.imshow(img_undist)
plt.title("Undistorted image")
plt.axis("off")
plt.show()

