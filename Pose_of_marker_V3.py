import pyrealsense2 as rs
import numpy as np
import cv2
#from Data_from_T265 import get_T265_pose, get_R_matrix_q

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

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



aruco_type = "DICT_5X5_100"

aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

aruco_parameters = cv2.aruco.DetectorParameters()

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

            r_vec, t_vec, trash = my_estimatePoseSingleMarkers(corners, 10.0, K_mtx_left, Distortion_left)
            
            if len(r_vec) > 0:
                print("r_vec: {}, t_vec: {}, trash: {}".format(r_vec, t_vec,trash))
                print("distance: {}".format(np.linalg.norm(t_vec[0])))
                for i in range(len(r_vec)):
                    # draw the ID of the detected marker over the top left corner
                    top_left_coner=corners[i][0][0].astype(int)
                    print("t_vec____: ",t_vec[i].flatten())
                    t_vec_flat=t_vec[i].flatten().astype(int)
                    cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(t_vec_flat[0],t_vec_flat[1],t_vec_flat[2])),(top_left_coner[0], top_left_coner[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                    cv2.aruco.drawDetectedMarkers(image_rgb, corners, ids, (0,255,0))
                    cv2.drawFrameAxes(image_rgb, K_mtx_left, Distortion_left, r_vec[i], t_vec[i], 5.0)
            cv2.imshow('Image_with_with_detections',image_rgb)
            
            key = cv2.waitKey(1)
            if key == 27 or key==ord('q'): # ESC or 'q'
                stop=False

finally:
    cv2.destroyAllWindows()
    pipe.stop()