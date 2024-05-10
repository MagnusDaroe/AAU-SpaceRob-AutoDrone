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

def get_T265_data(frames,print_data=False,confidence=True,print_time_stamp=True):
    """Get all data from the T265 camera and print it to the terminal if print_data is True
    
    Returns: 
        Lists: translation_xyz, rotation_xyzw, velocity, acceleration_xyz, angular velocity_xyz, angular acceleration_xyz, 
        Variable: tracker confidence, mapper confidence, frame number of the pose data, and the time stamp of the frame"""

    # Get the data from the latest frame in the pipeline
    pose = frames.get_pose_frame()
    data = pose.get_pose_data()

    if pose:
        time_stamp=pose.timestamp
        frame_number = pose.frame_number
        translation_xyz = [data.translation.x, data.translation.y, data.translation.z]
        rotation_xyzw = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
        velocity_xyz = [data.velocity.x, data.velocity.y, data.velocity.z]
        acceleration_xyz  = [data.acceleration.x, data.acceleration.y, data.acceleration.z]
        angular_velocity_xyz  = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
        angular_acceleration_xyz  = [data.angular_acceleration.x, data.angular_acceleration.y, data.angular_acceleration.z]
        tracker_confidence = data.tracker_confidence
        mapper_confidence = data.mapper_confidence


        # Print the data to the terminal if print_data is True
        if print_data:
            print("\n\nFrame #{}".format(frame_number))
            print("Position: x: {}, y: {}, z: {}".format(translation_xyz[0],translation_xyz[1],translation_xyz[2]))
            print("Rotation: x: {}, y: {}, z: {}, w: {}".format(rotation_xyzw[0],rotation_xyzw[1],rotation_xyzw[2],rotation_xyzw[3]))
            print("Velocity: x: {}, y: {}, z: {}".format(velocity_xyz[0],velocity_xyz[1],velocity_xyz[2]))
            print("Acceleration: x: {}, y: {}, z: {}".format(acceleration_xyz[0],acceleration_xyz[1],acceleration_xyz[2]))
            print("Angular Velocity: x: {}, y: {}, z: {}".format(angular_velocity_xyz[0],angular_velocity_xyz[1],angular_velocity_xyz[2]))
            print("Angular Acceleration: x: {}, y: {}, z: {}".format(angular_acceleration_xyz[0],angular_acceleration_xyz[1],angular_acceleration_xyz[2]))
            if confidence:
                print("Tracker Confidence: {}".format(tracker_confidence))
                print("Mapper Confidence: {}".format(mapper_confidence))
            if print_time_stamp:
                print("Time stamp: {}".format(pose.timestamp))
        return translation_xyz, rotation_xyzw, velocity_xyz, acceleration_xyz, angular_velocity_xyz, angular_acceleration_xyz, tracker_confidence, mapper_confidence, frame_number, time_stamp


def get_T265_pose(frames,print_data=False,print_confidence=True,print_time_stamp=True):
    """Get the pose data from the T265 camera and print it to the terminal if print_data is True
    
    Returns:
        Lists: translation_xyz, rotation_xyzw 
        Variable: tracker confidence, frame number of the pose data, and the time stamp of the frame"""

    # Get the data from the latest frame in the pipeline
    pose = frames.get_pose_frame()
    data = pose.get_pose_data()

    if pose:
        time_stamp=pose.timestamp
        frame_number = pose.frame_number
        translation_xyz = [data.translation.x, data.translation.y, data.translation.z]
        rotation_xyzw = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
        pose_confidence = data.tracker_confidence

        # Print the data to the terminal if print_data is True
        if print_data:
            print("Position: x: {}, y: {}, z: {}".format(translation_xyz[0],translation_xyz[1],translation_xyz[2]))
            print("Rotation: x: {}, y: {}, z: {}, w: {}".format(rotation_xyzw[0],rotation_xyzw[1],rotation_xyzw[2],rotation_xyzw[3]))
            print("Rotation: {}".format(rotation_xyzw))
            if print_confidence:
                print("Pose Confidence: {}".format(pose_confidence))
            if print_time_stamp==1:
                print("Time stamp: {}".format(time_stamp))
    return translation_xyz, rotation_xyzw, pose_confidence, frame_number, time_stamp


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

def quat_to_R(q_xyzw):
    """Convert a quaternion to a rotation matrix
    """
    # Define quaternion (w, x, y, z)
    #r = R.from_quat([q_xyzw[3],-q_xyzw[2],q_xyzw[0],-q_xyzw[1]])
    r =R.from_quat([q_xyzw[0],q_xyzw[1],q_xyzw[2],q_xyzw[3]])
    return r.as_matrix()

def q_to_extrinsic_roll_pitch_yaw(q_xyzw):
    """Convert a rotation matrix to extrinsic roll pitch yaw
    """
    x=q_xyzw[0]
    y=q_xyzw[1]
    z=q_xyzw[2]
    w=q_xyzw[3]
    yaw = math.atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z)
    pitch = math.asin(-2.0*(x*z - w*y))
    roll = math.atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z)
    return roll, pitch, yaw

def quaternion_to_euler(q_xyzw):
    """Convert a quaternion to Euler angles
    """
    q={'w':q_xyzw[3],'x':q_xyzw[0],'y':q_xyzw[1],'z':q_xyzw[2]}
    # Ensure the quaternion is normalized
    norm = math.sqrt(q['w']**2 + q['x']**2 + q['y']**2 + q['z']**2)
    q = {k: v/norm for k, v in q.items()}

    # Calculate the Euler angles
    roll = math.atan2(2.0*(q['y']*q['z'] + q['w']*q['x']), q['w']*q['w'] - q['x']*q['x'] - q['y']*q['y'] + q['z']*q['z'])
    pitch = math.asin(-2.0*(q['x']*q['z'] - q['w']*q['y']))
    yaw = math.atan2(2.0*(q['x']*q['y'] + q['w']*q['z']), q['w']*q['w'] + q['x']*q['x'] - q['y']*q['y'] - q['z']*q['z'])

    return roll, pitch, yaw

def quaternion_to_euler_V2(q_xyzw):
    """Convert a quaternion to Euler angles
    """
    q={'w':q_xyzw[3],'x':q_xyzw[0],'y':q_xyzw[1],'z':q_xyzw[2]}   
    norm = math.sqrt(q['w']**2 + q['x']**2 + q['y']**2 + q['z']**2)
    q = {k: v/norm for k, v in q.items()}

    # Calculate the Euler angles
    roll = math.atan2(2.0*(q['y']*q['z'] + q['w']*q['x']), q['w']*q['w'] - q['x']*q['x'] - q['y']*q['y'] + q['z']*q['z'])
    pitch = math.asin(-2.0*(q['x']*q['z'] - q['w']*q['y']))
    yaw = math.atan2(2.0*(q['x']*q['y'] + q['w']*q['z']), q['w']*q['w'] + q['x']*q['x'] - q['y']*q['y'] - q['z']*q['z'])

    # Adjust the angles to be within the range of 0 to 2*pi
    roll = roll % (2.0 * math.pi)
    pitch = pitch % (2.0 * math.pi)
    yaw = yaw % (2.0 * math.pi)

    return roll, pitch, yaw

def q_to_RPY(qrot_xyzw):
    w = qrot_xyzw[3]
    x = -qrot_xyzw[2]
    y = qrot_xyzw[0]
    z = -qrot_xyzw[1]

    pitch =  -math.asin(2.0 * (x*z - w*y)) * 180.0 / math.pi
    roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / math.pi
    yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / math.pi

    print("RPY * [deg]:[ {0:.7f}, {1:.7f}, {2:.7f}]".format(roll, pitch, yaw))

try:
    for _ in range(2000):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Get the pose from the T265 camera
        translation_xyz, rotation_xyzw, velocity_xyz, acceleration_xyz, angular_velocity_xyz, angular_acceleration_xyz, tracker_confidence, mapper_confidence, frame_number, time_stamp = get_T265_data(frames,print_data=True)

        # Get the R matrix from the quaternion
        R_mtx=get_R_matrix_q(rotation_xyzw)
        print("R_mtx egen: \n",R_mtx)
        R_mtx_q=quat_to_R(rotation_xyzw)
        print("R_mtx quat: \n",R_mtx_q)
        R_roty_180=np.array([[-1,0,0],[0,1,0],[0,0,-1]])
        euler_angles = R.from_matrix(inv(R_roty_180)@R_mtx).as_euler('zyx', degrees=True)
        
        print("Euler angles: ",euler_angles)
        roll, pitch, yaw = q_to_extrinsic_roll_pitch_yaw(rotation_xyzw)
        print("V1 - Roll: {}, Pitch: {}, Yaw: {}".format(np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)))
        
        roll, pitch, yaw = quaternion_to_euler(rotation_xyzw)
        print("V2 - Roll: {}, Pitch: {}, Yaw: {}".format(np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)))
        
        roll, pitch, yaw =quaternion_to_euler_V2(rotation_xyzw)
        print("V2 - Roll: {}, Pitch: {}, Yaw: {}".format(np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)))
        
        q_to_RPY(rotation_xyzw)

        time.sleep(0.1)



finally:
    pipe.stop()