import pyrealsense2 as rs
import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as Rot
import cv2
#from Data_from_T265 import get_T265_pose, get_R_matrix_q

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
#T_C_LEFT = np.eye(4)
#T_C_LEFT[:3, :3] = np.array([[1 , 0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
#T_C_LEFT[:3, 3] = np.array([left_to_center_cm, 0, lense_to_center_cm])

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
    #R_2=R.from_quat([qrot_xyzw[3],-qrot_xyzw[2],qrot_xyzw[0],-qrot_xyzw[1]])
    return T_1

def new_get_T_matrix_q(qrot_xyzw,translation_xyz):

    R = Rot.from_quat([qrot_xyzw[0],qrot_xyzw[1],qrot_xyzw[2],qrot_xyzw[3]])
    R_mtx=R.as_matrix()
    T_mtx = np.eye(4)
    T_mtx[:3, :3] = R_mtx
    T_mtx[:3, 3] = translation_xyz
    return T_mtx

def new_get_R_matrix_q(qrot_xyzw,translation_xyz):

    R = Rot.from_quat([qrot_xyzw[0],qrot_xyzw[1],qrot_xyzw[2],qrot_xyzw[3]])
    R_mtx=R.as_matrix()
    return R_mtx

def get_R_matrix_q(qrot_xyzw):
    """Convert a quaternion to a rotation matrix
    """
    # Define quaternion (w, x, y, z)
    w = qrot_xyzw[3]
    x = qrot_xyzw[0]
    y = qrot_xyzw[1]
    z = qrot_xyzw[2]#-qrot_xyzw[2]
    R_1 = np.array([[-(1-2*y*y-2*z*z), 2*x*y-2*z*w, -(2*x*z+2*y*w)],
                    [-(2*x*y+2*z*w), 1-2*x*x-2*z*z, -(2*y*z-2*x*w)],
                    [-(2*x*z-2*y*w), 2*y*z+2*x*w, -(1-2*x*x-2*y*y)]])
    #R_2=R.from_quat([qrot_xyzw[3],-qrot_xyzw[2],qrot_xyzw[0],-qrot_xyzw[1]])
    return R_1

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


# Define the type of aruco markers to detect (number of square and size of the markers)
aruco_type = "DICT_5X5_100" # 5x5 markers of 100mm

# Create the aruco dictionary and the parameters to detect the markers
aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
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
            x_max=image_left.shape[1]
            y_max=image_left.shape[0]

            #crop to 300x300 pixels around the center
            x_center=int(x_max/2)
            y_center=int(y_max/2)
            x_min=x_center-150
            x_max=x_center+150
            y_min=y_center-150
            y_max=y_center+150
            #print("x_min: {}, x_max: {}, y_min: {}, y_max: {}".format(x_min,x_max,y_min,y_max))
            #image_left=image_left[y_min:y_max,x_min:x_max]
            
            MIN_VAL=350
            MAX_VAL=450

            # make rgb image from grayscale
            image_rgb = cv2.cvtColor(image_left, cv2.COLOR_GRAY2RGB)
            
            corners, ids, rejected =detector.detectMarkers(image_left, K_mtx_left, Distortion_left)
            #print the type the corners variable
            corners_lst=[]
            print("corners: ",corners)
            for i,arr in enumerate(corners):
                if arr[0].min()>MIN_VAL and arr[0].max()<MAX_VAL:
                    corners_lst.append(arr[0])
            print("coners: ",corners)
            if len(corners_lst)>0:
                print("coners[0]: ",corners[0])
            #print("coners_new: ",coners_new)
            #print("corners_lst: ",corners_lst)
            #print("corners_tuple:",tuple(corners_lst))
            #print("len of corners_lst:",len(corners_lst))
            #print("len of corners_tuple:",len(tuple(corners_lst)))
            #corners=tuple(corners_lst)
            #print("corners: ",corners)
            corners_new=tuple(corners_lst)
            print("corners_new",corners_new)

            r_vec, t_vec, trash = my_estimatePoseSingleMarkers(corners_new, 10.0, K_mtx_left, Distortion_left)

            #get the pose data from the T265 camera
            translation_xyz, rotation_xyzw = get_pose_data(frames)
            pos_cm=np.array(translation_xyz)*100
            pos_cm=pos_cm.astype(int)
            cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(pos_cm[0],pos_cm[1],pos_cm[2])),(20, 20), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
            
            T_q=get_T_matrix_q(rotation_xyzw,translation_xyz)
            
            if len(r_vec) > 0:
                #print("r_vec: {}, t_vec: {}, trash: {}".format(r_vec, t_vec,trash))
                #print("distance: {}".format(np.linalg.norm(t_vec[0])))
                for i in range(len(r_vec)):
                    # draw the ID of the detected marker over the top left corner
                    top_left_coner=corners[i][0][0].astype(int)
                    #print("t_vec____: ",t_vec[i].flatten())
                    t_vec_flat=t_vec[i].flatten().astype(int)
                    P_center=T_C_LEFT @np.array([[t_vec_flat[0]],[t_vec_flat[1]],[t_vec_flat[2]],[1]])
                    ####P_center_1x3=-np.array([P_center[0],P_center[1],P_center[2]])
                    P_center_1x3=np.array([P_center[0],P_center[1],P_center[2],[-1]])
                    #print("P_center1x3: ",P_center_1x3)
                    ####P_marker_pos=R_q@P_center_1x3+np.array([[translation_xyz[0]],[translation_xyz[1]],[translation_xyz[2]]])*100
                    P_marker_pos=T_q@(P_center_1x3)
                    
                    P_center_1x3_round=P_center_1x3.round(0)
                    #print("P_center: ",P_center[0][0],P_center[1][0],P_center[2][0])
                    P_marker_pos=P_marker_pos.round(0)
                    cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(P_center_1x3_round[0][0],P_center_1x3_round[1][0],P_center_1x3_round[2][0])),(top_left_coner[0], top_left_coner[1] - 50), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0, 0), 2)
                    cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(P_marker_pos[0][0],P_marker_pos[1][0],P_marker_pos[2][0])),(top_left_coner[0], top_left_coner[1] - 30), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
                    cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(t_vec_flat[0],t_vec_flat[1],t_vec_flat[2])),(top_left_coner[0], top_left_coner[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                    
                    cv2.aruco.drawDetectedMarkers(image_rgb, corners, ids, (0,255,0))
                    cv2.drawFrameAxes(image_rgb, K_mtx_left, Distortion_left, r_vec[i], t_vec[i], 5.0)
            cv2.rectangle(image_rgb, (MIN_VAL, MIN_VAL), (MAX_VAL, MAX_VAL), (255, 0, 0), 2)
            cv2.imshow('Image_with_with_detections',image_rgb)
            
            key = cv2.waitKey(1)
            if key == 27 or key==ord('q'): # ESC or 'q'
                stop=False

finally:
    cv2.destroyAllWindows()
    pipe.stop()