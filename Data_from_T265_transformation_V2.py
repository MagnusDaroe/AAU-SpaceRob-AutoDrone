import pyrealsense2 as rs
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
from numpy.linalg import inv

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build configuation object and request pose data from the T265
cfg = rs.config()
    # Enable the pose stream
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested configuration
pipe.start(cfg)



def get_T265_pose(frames,print_data=False,print_confidence=True,print_time_stamp=True):
    """Get the pose data from the T265 camera and print it to the terminal if print_data is True
    
    Returns:
        Lists: translation_xyz_mm, rotation_xyzw 
        Variable: tracker confidence, frame number of the pose data, and the time stamp of the frame"""

    # Get the data from the latest frame in the pipeline
    pose = frames.get_pose_frame()
    data = pose.get_pose_data()

    if pose:
        time_stamp=pose.timestamp
        frame_number = pose.frame_number
        translation_xyz_mm = [data.translation.x*1000, data.translation.y*1000, data.translation.z*1000] # in mm
        rotation_xyzw = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
        pose_confidence = data.tracker_confidence

        # Print the data to the terminal if print_data is True
        if print_data:
            print("Position: x: {}, y: {}, z: {}".format(translation_xyz_mm[0],translation_xyz_mm[1],translation_xyz_mm[2]))
            print("Rotation: x: {}, y: {}, z: {}, w: {}".format(rotation_xyzw[0],rotation_xyzw[1],rotation_xyzw[2],rotation_xyzw[3]))
            print("Rotation: {}".format(rotation_xyzw))
            if print_confidence:
                print("Pose Confidence: {}".format(pose_confidence))
            if print_time_stamp==1:
                print("Time stamp: {}".format(time_stamp))
    return translation_xyz_mm, rotation_xyzw, pose_confidence, frame_number, time_stamp

def q_to_RPY(qrot_xyzw):
    w = qrot_xyzw[3]
    x = -qrot_xyzw[2]
    y = qrot_xyzw[0]
    z = -qrot_xyzw[1]

    pitch =  -math.asin(2.0 * (x*z - w*y)) #* 180.0 / math.pi
    roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) #* 180.0 / math.pi
    yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) #* 180.0 / math.pi

    return roll, pitch, yaw
    

def RPY_to_R(r,p,y):
    """Convert excentric roll, pitch, and yaw to a rotation matrix
    """
       # Calculate trigonometric values
    cos_r = np.cos(r)
    sin_r = np.sin(r)
    cos_p = np.cos(p)
    sin_p = np.sin(p)
    cos_y = np.cos(y)
    sin_y = np.sin(y)
    
    # Compute rotation matrix
    R_x = np.array([[1, 0, 0],
                       [0, cos_r, -sin_r],
                       [0, sin_r, cos_r]])
    
    R_y = np.array([[cos_p, 0, sin_p],
                        [0, 1, 0],
                        [-sin_p, 0, cos_p]])
    
    R_z = np.array([[cos_y, -sin_y, 0],
                      [sin_y, cos_y, 0],
                      [0, 0, 1]])
    
    R_mtx= R_z @ R_y @ R_x
    return R_mtx

def R_to_euler_angles(r_mtx_global):
    """Convert a rotation matrix to euler angles
    """
    #get extrinsic euler angles
    sy = math.sqrt(r_mtx_global[0,0] * r_mtx_global[0,0] +  r_mtx_global[1,0] * r_mtx_global[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(r_mtx_global[2,1] , r_mtx_global[2,2])
        y = math.atan2(-r_mtx_global[2,0], sy)
        z = math.atan2(r_mtx_global[1,0], r_mtx_global[0,0])
    else :
        x = math.atan2(-r_mtx_global[1,2], r_mtx_global[1,1])
        y = math.atan2(-r_mtx_global[2,0], sy)
        z = 0
    return [x,y,z]

def R_to_T(R, t_xyz):
    """Convert a rotation matrix and translation vector to a transformation matrix
    """
    """
    t_vec=np.array([t_xyz[0],t_xyz[1],t_xyz[2]])
    T_mtx = np.eye(4)
    T_mtx[:3,:3] = R
    T_mtx[:3,3] = [-t_vec[2], t_vec[0], -t_vec[1]]
    t_xyz=np.array(t_xyz)
    print("t_xyz[0]",t_xyz[2])
    print("R[0,0]",R[0,0])"""

    T_mtx=np.array([[R[0,0], R[0,1], R[0,2], -1*t_xyz[2]],
                    [R[1,0], R[1,1], R[1,2], t_xyz[0]],
                    [R[2,0], R[2,1], R[2,2], -1*t_xyz[1]],
                    [0, 0, 0, 1]])
    return T_mtx
def R_to_euler_angles_V2(r_mtx_global):
    """use scipy to convert a rotation matrix to euler angles
    """
    
    r = R.from_matrix(r_mtx_global)
    euler_xyz=r.as_euler('xyz', degrees=False)
    return euler_xyz
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
    z = qrot_xyzw[2]
    Transfromation_mtx = np.array([[-(1-2*y*y-2*z*z), 2*x*y-2*z*w, -(2*x*z+2*y*w),x_t],
                    [-(2*x*y+2*z*w), 1-2*x*x-2*z*z, -(2*y*z-2*x*w),y_t],
                    [-(2*x*z-2*y*w), 2*y*z+2*x*w, -(1-2*x*x-2*y*y),z_t],
                    [0,0,0,1]])
    
    return Transfromation_mtx
try:
    for _ in range(2000):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Get the pose from the T265 camera
        print("\n\n\n")
        translation_xyz_mm, rotation_xyzw, pose_confidence, frame_number, time_stamp = get_T265_pose(frames)
        roll, pitch, yaw= q_to_RPY(rotation_xyzw)
        
        R_mtx=RPY_to_R(roll,pitch,yaw)
        T_cam_pose=R_to_T(R_mtx,translation_xyz_mm)
        print("R_mtx: \n",R_mtx)
        T_q=get_T_matrix_q(rotation_xyzw,translation_xyz_mm)
        print("R_q: \n",T_q[:3,:3])

        T_start_ref=np.array([[1,0,0,0],
                              [0,-1,0,0],
                              [0,0,-1,0],
                              [0,0,0,1]])
        angle=-75
        T_ref_cam=np.array([[np.cos(np.deg2rad(angle)),0,np.sin(np.deg2rad(angle)),0],
                                    [0,1,0,0],
                                    [-np.sin(np.deg2rad(angle)),0,np.cos(np.deg2rad(angle)),0],
                                    [0,0,0,1]])
        #T_ref_cam@
        #T_mtx=T_start_ref@T_cam_pose
        T_mtx=T_cam_pose
        R_new=inv(T_ref_cam[:3,:3]@inv(R_mtx))
        print("R_new: \n",R_new)
        
        #P_center 
        #backside center of T265 [01.54,09.10,-05.75,1] 
        #FC to backside [153.223,0,-86.6070,1]
        
        P_C=np.array([-153.223+01.54,9.1,-1*(-05.75-86.6070)])
        print("P_C: \n",P_C)
        P_FC=np.dot(R_new,P_C)
        print("P_FC: \n",P_FC)
        T_new=np.array([[R_new[0,0], R_new[0,1], R_new[0,2], 0],
                        [R_new[1,0], R_new[1,1], R_new[1,2],0],
                        [R_new[2,0], R_new[2,1], R_new[2,2], 0],
                        [0, 0, 0, 1]])
        T_C=np.array([[1,0,0,P_C[0]],
                      [0,1,0,P_C[1]],
                      [0,0,1,P_C[2]],
                      [0,0,0,1]])
        P_FC_2=T_new@T_C
        print("P_FC_2: \n",[P_FC_2[0,3],P_FC_2[1,3],P_FC_2[2,3]])

        print("T_mtx: \n",T_mtx)
        print("RPY * [deg]:[ {0:.7f}, {1:.7f}, {2:.7f}]".format(np.rad2deg(roll), np.rad2deg(pitch)+75, np.rad2deg(yaw)))
        t_vec=np.array([T_mtx[0,3],T_mtx[1,3],T_mtx[2,3]])-P_C
        t_vec=t_vec*0.1
        euler_xyz=R_to_euler_angles_V2(T_mtx[:3,:3])
        print("euler_xyz: X: {}, Y: {}, Z: {}".format(round(np.rad2deg(euler_xyz[0]),2),round(np.rad2deg(euler_xyz[1]),2),round(np.rad2deg(euler_xyz[2]),2)))
        print("pos - x: {}, y: {}, z: {}".format(round(t_vec[0]+P_FC[0]*0.1,2),round(t_vec[1]+P_FC[1]*0.1,2),round(t_vec[2]+P_FC[2]*0.1,2)))
        time.sleep(0.1)
        


finally:
    pipe.stop()