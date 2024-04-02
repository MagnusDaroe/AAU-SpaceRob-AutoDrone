#Get image from T265 camera with out fisheye distortion

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2


# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build configuation object and request pose data from the T265
#cfg = rs.config()

#cfg.enable_stream()

# Start streaming with requested configuration
pipe.start()

stop = True
try:
    while stop==True:
        frames = pipe.wait_for_frames()
        left_frame = frames.get_fisheye_frame(1)
        right_frame = frames.get_fisheye_frame(2)
        if left_frame:
            image_left = np.asanyarray(left_frame.get_data())
            image_right = np.asanyarray(right_frame.get_data())
            images = np.hstack((image_left, image_right))
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            key = cv2.waitKey(1)
            if key == 27 or key==ord('q'): # ESC or 'q'
                stop=False

finally:
    pipe.stop()
