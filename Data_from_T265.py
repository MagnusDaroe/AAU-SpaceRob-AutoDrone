import pyrealsense2 as rs
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

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

def quat_to_euler(q_xyzw):
    """Convert a quaternion to Euler angles
    """
    # Define quaternion (w, x, y, z)
    r = R.from_quat([q_xyzw[3],-q_xyzw[2],q_xyzw[0],-q_xyzw[1]])
    euler = r.as_euler('zyx', degrees=True)
    return euler

def q_to_eul(qrot_xyzw):#print_RPY=False
    """Convert a quaternion to Euler angles
    """
    w = qrot_xyzw[3]
    x = -qrot_xyzw[2]
    y = qrot_xyzw[0]
    z = -qrot_xyzw[1]#-qrot_xyzw[2]

    # Convert quaternion to Euler angles
    roll = -math.atan2(2.0 * (y*z + w*x), w*w - x*x - y*y + z*z)
    pitch = -math.asin(-2.0 * (x*z - w*y))
    yaw = math.atan2(2.0 * (x*y + w*z), w*w + x*x - y*y - z*z)

    # Convert from radians to degrees
    roll = np.degrees(roll)
    pitch = np.degrees(pitch)
    yaw = np.degrees(yaw)
    """
    pitch =  -math.asin(2.0 * (x*z - w*y)) * 180.0 / math.pi;
    roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / math.pi;
    yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / math.pi;"""

    list_RPY=[roll, pitch, yaw]
    #if print_RPY:
    #        print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(roll, pitch, yaw))

    return list_RPY

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

def get_R_matrix_eul(eul):
    """Convert Euler angles to a rotation matrix
    """
    # Define Euler angles (roll, pitch, yaw)
    roll = eul[0]
    pitch = eul[1]
    yaw = eul[2]

    # Convert Euler angles to a rotation matrix
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]])

    R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                    [0, 1, 0],
                    [-math.sin(pitch), 0, math.cos(pitch)]])

    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])

    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

try:
    for _ in range(5000):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        #get_T265_data(frames,1,1)

        translation_xyz, rotation_xyzw,pose_confidence, frame_number, time_stamp=get_T265_pose(frames,0)
        rot=quat_to_euler(rotation_xyzw)
        #rot=q_to_eul(rotation_xyzw)
        print("Rot: roll: {}, pitch: {}, raw: {}".format(rot[0].round(2),rot[1].round(2),rot[2].round(2)))
        # Print the data
        print("Rot: roll: {}, pitch: {}, raw: {}".format(rot[0].round(2), rot[1].round(2), rot[2].round(2)))

        # Save data to CSV file
        #with open("pose_data.csv", mode='a') as file:
            #np.savetxt(file, [rot], delimiter=",", fmt='%.2f')


        
        R_1=get_R_matrix_q(rotation_xyzw)
        #print("R1_matrix_q: \n{}".format(R_1))
        #print("R_matrix_eul: \n{}".format(get_R_matrix_eul(q_to_eul(rotation_xyzw))))
        pos_cm=np.array(translation_xyz)*100
        #reshap to vector
        #print("Pose in cm: x: {}, y: {}, z: {}".format(translation_xyz[0]*100,translation_xyz[1]*100,translation_xyz[2]*100))
        #print("Pose in cm: x: {}, y: {}, z: {}".format(pos_cm[0],pos_cm[1],pos_cm[2]))
        pos_cm=pos_cm.astype(int).reshape(3,-1)
        #print(pos_cm)
        R_g_to_l=get_R_matrix_q(rotation_xyzw)

        R_left_to_l=np.array([[1,0,0],
                              [0,1,0],
                              [0,0,1]])
        
        #delay of 0.1 seconds
        time.sleep(0.1)




        



finally:
    pipe.stop()