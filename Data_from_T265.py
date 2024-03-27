import pyrealsense2 as rs
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
    
    Returns: translation, rotation, velocity, acceleration, angular velocity, angular acceleration, 
    tracker confidence, mapper confidence, frame number of the pose data, and the time stamp of the frame"""

    # Get the data from the latest frame in the pipeline
    pose = frames.get_pose_frame()
    data = pose.get_pose_data()

    if pose:
        time_stamp=pose.timestamp
        frame_number = pose.frame_number
        translation = data.translation
        rotation = data.rotation
        velocity = data.velocity
        acceleration = data.acceleration
        angular_velocity = data.angular_velocity
        angular_acceleration = data.angular_acceleration
        tracker_confidence = data.tracker_confidence
        mapper_confidence = data.mapper_confidence

        # Print the data to the terminal if print_data is True
        if print_data:
            print("Frame #{}".format(frame_number))
            print("Position: {}".format(translation))
            print("Velocity: {}".format(velocity))
            print("Acceleration: {}\n".format(acceleration))
            print("Rotation: {}".format(rotation))
            print("Angular Velocity: {}".format(angular_velocity))
            print("Angular Acceleration: {}".format(angular_acceleration))
            if confidence:
                print("Tracker Confidence: {}".format(tracker_confidence))
                print("Mapper Confidence: {}".format(mapper_confidence))
            if print_time_stamp:
                print("Time stamp: {}".format(pose.timestamp))
        return translation, rotation, velocity, acceleration, angular_velocity, angular_acceleration, tracker_confidence, mapper_confidence, frame_number, time_stamp



def get_T265_pose(frames,print_data=False,print_confidence=True,print_time_stamp=True):
    """Get the pose data from the T265 camera and print it to the terminal if print_data is True
    
    Returns the translation, rotation, pose confidence, frame number of the pose data, and the time stamp of the frame"""

    # Get the data from the latest frame in the pipeline
    pose = frames.get_pose_frame()
    data = pose.get_pose_data()

    time_stamp=pose.timestamp
    frame_number = pose.frame_number
    translation = data.translation
    rotation = data.rotation
    pose_confidence = data.tracker_confidence

    # Print the data to the terminal if print_data is True
    if print_data:
        print("Frame #{}".format(frame_number))
        print("Position: {}".format(translation))
        print("Rotation: {}".format(rotation))
        if print_confidence:
            print("Pose Confidence: {}\n\n".format(pose_confidence))
        if print_time_stamp==1:
            print("Time stamp: {}".format(time_stamp))
    return translation, rotation, pose_confidence, frame_number, time_stamp


print_all_data = False

try:
    for _ in range(5000):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        if print_all_data:
            get_T265_data(frames,print_data=True,confidence=True)
        else:
            get_T265_pose(frames,0)

        


finally:
    pipe.stop()