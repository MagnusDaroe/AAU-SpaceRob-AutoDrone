"""
opencv-contrib-python         4.7.0.72
opencv-python                 4.8.1.78
"""

import pyrealsense2 as rs
from numpy.linalg import inv
#from scipy.spatial.transform import Rotation as Rot
import numpy as np
import cv2
import os

ARUCO_DICT = cv2.aruco.DICT_5X5_100  # Dictionary ID


#calibration file
CALIBRATION_FILE = "Calibrate_camera_2\calib_new_1.npz"



with np.load(CALIBRATION_FILE) as X:
    ret,mtx, dist = [X[i] for i in ("ret","mtx", "dist")]

left_to_center_mm=-32.00
lense_to_center_mm=6.55

left_to_center_cm=left_to_center_mm/10
lense_to_center_cm=lense_to_center_mm/10

theta=-np.pi
# Define the Transformation matrix from the left camera sensor to pose center of the T265
T_C_LEFT=np.array([[1 , 0,0,left_to_center_cm],
                [0,np.cos(theta),-np.sin(theta),0],
                [0,np.sin(theta),np.cos(theta),lense_to_center_cm],
                [0,0,0,1]])


def get_T_matrix_q(qrot_xyzw,translation_xyz):
    """Convert a quaternion to a rotation matrix
    """
    x_t=translation_xyz[0]*100
    y_t=translation_xyz[1]*100
    z_t=translation_xyz[2]*100
    
    # Define quaternion (w, x, y, z)
    w = qrot_xyzw[3]
    x = qrot_xyzw[0]
    y = qrot_xyzw[1]
    z = qrot_xyzw[2]#-qrot_xyzw[2]
    T_1 = np.array([[-(1-2*y*y-2*z*z), 2*x*y-2*z*w, -(2*x*z+2*y*w),x_t],
                    [-(2*x*y+2*z*w), 1-2*x*x-2*z*z, -(2*y*z-2*x*w),y_t],
                    [-(2*x*z-2*y*w), 2*y*z+2*x*w, -(1-2*x*x-2*y*y),z_t],
                    [0,0,0,1]])
    return T_1

def get_pose_data(frames):
    """Get the pose data from the T265 camera
    """
    # Get the data from the latest frame in the pipeline
    pose = frames.get_pose_frame()
    data = pose.get_pose_data()
    if pose:
        time_stamp=pose.timestamp
        frame_number = pose.frame_number
        translation_xyz = [data.translation.x, data.translation.y, data.translation.z]
        rotation_xyzw = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
        pose_confidence = data.tracker_confidence
    return translation_xyz, rotation_xyzw#, pose_confidence, frame_number, time_stamp


def get_marker_pose(image,output_image,mtx,dist):
    t_vec=[]
    r_vec=None
    marker_corners, marker_ids, rejectedCandidates = detector.detectMarkers(image)

    # SUB PIXEL DETECTION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    for corner in marker_corners:
        cv2.cornerSubPix(image, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

    if marker_ids is not None and len(marker_ids) > 0: # If at least one marker is detected
        r_vec,t_vec,_objPoints = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 10 , mtx, dist)
        #if r_vec is not None:
        for i in range(len(r_vec)):
            cv2.aruco.drawDetectedMarkers(output_image, marker_corners, marker_ids)
            cv2.drawFrameAxes(output_image, mtx, dist, r_vec[i], t_vec[i], length=10, thickness=2)
            
    return r_vec,t_vec,np.array(marker_corners)

dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, params)

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Start streaming with requested configuration
pipe.start()

# Declare the variable stop to stop the while loop
stop = True

try:
    # The while loop will run until the variable stop is set to False by pressing the 'q' or 'ESC' key
    while stop==True:
        # Get the frames from the T265 camera, when the data is available
        frames = pipe.wait_for_frames()
        left_frame = frames.get_fisheye_frame(1)

        # If the frame is available, get the image from the left camera and show it undistorted in a window
        if left_frame:          
            image_left = np.asanyarray(left_frame.get_data())

            h,  w = image_left.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
            img_undist = cv2.undistort(image_left, mtx, dist, None, newcameramtx)
            img_undis_2 = cv2.undistort(src = image_left, cameraMatrix = mtx, distCoeffs = dist)
            img_undist=img_undis_2
            image_rgb = cv2.cvtColor(img_undist, cv2.COLOR_GRAY2RGB)

            #get the pose data from the T265 camera
            translation_xyz, rotation_xyzw = get_pose_data(frames)
            pos_cm=np.array(translation_xyz)*100
            pos_cm=pos_cm.astype(int)

            T_q=get_T_matrix_q(rotation_xyzw,translation_xyz)

            cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(pos_cm[0],pos_cm[1],pos_cm[2])),(20, 20), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
            r_vec,t_vec_,coner=get_marker_pose(img_undist,image_rgb,mtx,dist)
            #print("coner:\n",coner)
            print("t_vec_:\n",t_vec_)
            if r_vec is not None:
                for i in range(len(r_vec)):
                    top_left_coner=coner[i][0][0].astype(int)
                    #t_vec_[i]=t_vec_[i]*100
                    t_vec_flat=t_vec_[i].round(0).flatten().astype(int)

                    P_center=T_C_LEFT @np.array([[t_vec_flat[0]],[t_vec_flat[1]],[t_vec_flat[2]],[1]])
                    
                    P_center_1x3=np.array([P_center[0],P_center[1],P_center[2],[-1]])

                    P_marker_pos=T_q@(P_center_1x3)
                    
                    P_center_1x3_round=P_center_1x3.round(0)
                    P_marker_pos=P_marker_pos.round(0)
                    
                    cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(t_vec_flat[0],t_vec_flat[1],t_vec_flat[2])),(coner[0].flatten()[0].astype(int),coner[0].flatten()[1].astype(int)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                    cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(P_marker_pos[0][0],P_marker_pos[1][0],P_marker_pos[2][0])),(top_left_coner[0], top_left_coner[1] - 30), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
                    #cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(P_marker_pos[0][0],P_marker_pos[1][0],P_marker_pos[2][0])),(coner[0].flatten()[0].astype(int),coner[0].flatten()[1].astype(int)-30), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)

            image_rgb_sz=image_rgb[300:-300,300:-300]
            
            #print("image_rgb.shape",image_rgb_sz.shape)
            cv2.imshow('Image_with_with_detections',image_rgb_sz)
            cv2.imshow('Image_rgb',image_rgb)
            #cv2.imshow('Image',img_undist)
            #cv2.imshow('Image 2',img_undis_2[200:-200,200:-200])
            
            key = cv2.waitKey(1)
            if key == 27 or key==ord('q'): # ESC or 'q'
                stop=False

finally:
    cv2.destroyAllWindows()
    pipe.stop()