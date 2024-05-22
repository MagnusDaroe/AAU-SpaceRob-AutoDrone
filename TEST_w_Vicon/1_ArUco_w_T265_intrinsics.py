import pyrealsense2 as rs
import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as Rot
import cv2
import time

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())

    Source: https://github.com/Menginventor/aruco_example_cv_4.8.0/blob/main/pose_estimate.py
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


# Define the type of aruco markers to detect (number of squares in the marker)
ARUCO_DICT=cv2.aruco.DICT_5X5_100
#ARUCO_DICT=cv2.aruco.DICT_4X4_100

MAKER_SIZE=0.1515152 #meters

# Create the aruco dictionary and the parameters to detect the markers
aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
aruco_parameters = cv2.aruco.DetectorParameters()

# Create the detector with the dictionary and parameters
detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_parameters)

# Functions to get the camera matrix and distortion coefficients from the intrinsics
def camera_matrix(intrinsics):
    """
    Returns a camera matrix K from librealsense intrinsics
    """
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

def fisheye_distortion(intrinsics):
    """
    Returns the fisheye distortion from librealsense intrinsics
    """
    return np.array(intrinsics.coeffs[:4])

def t_FCvicon_marker(t_vec_flat_meter):
        """Transform the t_vec from the camera frame to the vicon frame
        """
        y_left_Cam=32.000 #mm

        P_Cam_marker=np.array([[t_vec_flat_meter[2]*1000],[t_vec_flat_meter[0]*1000+y_left_Cam],[t_vec_flat_meter[1]*1000]])
        theta=np.deg2rad(75)
        rotz_75=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        P_CamFlat_marker=rotz_75@P_Cam_marker
        P_FC_Cam=np.array([[154.763],[-9.100],[92.357]]) #mm
        P_FC_marker=P_FC_Cam+P_CamFlat_marker
        t_FCvicon_marker_=np.array([-P_FC_marker[0][0],P_FC_marker[1][0],-P_FC_marker[2][0]])
        return t_FCvicon_marker_ 

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Start streaming with requested configuration
pipe.start()

# Declare the variable stop to stop the while loop
stop = True

try:
    # Get the intrinsics of the left camera from the T265 camera and calculate the undistort_rectify map and the max disparity
    intrinsics = pipe.get_active_profile().get_stream(rs.stream.fisheye, 1).as_video_stream_profile().get_intrinsics()

    K_mtx_left  = camera_matrix(intrinsics)
    Distortion_left  = fisheye_distortion(intrinsics)

    # The while loop will run until the variable stop is set to False by pressing the 'q' or 'ESC' key
    while stop==True:
        # Get the frames from the T265 camera, when the data is available
        frames = pipe.wait_for_frames()
        left_frame = frames.get_fisheye_frame(1)

        # If the frame is available, get the image from the left camera and show it undistorted in a window
        if left_frame:
            image_left = np.asanyarray(left_frame.get_data())
            # make rgb image from grayscale
            image_rgb = cv2.cvtColor(image_left, cv2.COLOR_GRAY2RGB)
            
            corners, ids, rejected =detector.detectMarkers(image_left, K_mtx_left, Distortion_left)

            r_vec, t_vec, trash = my_estimatePoseSingleMarkers(corners, MAKER_SIZE, K_mtx_left, Distortion_left)

            if len(r_vec) > 0:
                #convert all t_vec to mm
                for i in range(len(r_vec)):
                    # draw the ID of the detected marker over the top left corner
                    top_left_coner=corners[i][0][0].astype(int)
                    t_vec_flat=t_vec[i].flatten()

                    cv2.putText(image_rgb, str("Cam to Marker [cm] x: {}, y: {}, z: {}".format(round(t_vec_flat[0]*100,3),round(t_vec_flat[1]*100,3),round(t_vec_flat[2]*100,3))),(20, 20), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
                    print("\nCam to Marker [mm] x: {}, y: {}, z: {}".format(round(t_vec_flat[0]*1000,3),round(t_vec_flat[1]*1000,3),round(t_vec_flat[2]*1000,3)))
                    
                    t_FCvicon_marker_=t_FCvicon_marker(t_vec_flat)
                    print("t_FCvicon_marker [mm] ",[round(t_FCvicon_marker_[0],3),round(t_FCvicon_marker_[1],3),round(t_FCvicon_marker_[2],3)]) #mm
                    cv2.putText(image_rgb, str("FC_Vicon to Marker [cm] x: {}, y: {}, z: {}".format(round(t_FCvicon_marker_[0]*0.1,3),round(t_FCvicon_marker_[1]*0.1,3),round(t_FCvicon_marker_[2]*0.1,3))),(20, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)

                    cv2.aruco.drawDetectedMarkers(image_rgb, corners, ids, (0,255,0))
                    cv2.drawFrameAxes(image_rgb, K_mtx_left, Distortion_left, r_vec[i], t_vec[i], 0.1)
            cv2.imshow('Image_with_with_detections',image_rgb)
            key = cv2.waitKey(1)
            if key == 27 or key==ord('q'): # ESC or 'q'
                stop=False

finally:
    cv2.destroyAllWindows()
    pipe.stop()