#Get image from T265 camera and display it, if 

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2
import os

path = "_Calibrate_camera_2\Img_for_calibration_2"

#Create folder for images if it does not exist
if not os.path.exists(path):
    os.makedirs(path)

frame_counter = 143#0  # Initialize frame counter

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Start streaming with requested configuration
pipe.start()

stop = True
try:
    while stop==True:
        frames = pipe.wait_for_frames()
        left_frame = frames.get_fisheye_frame(1)
        if left_frame:
            image_left = np.asanyarray(left_frame.get_data())

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', image_left)
            key = cv2.waitKey(1)
            if key == ord('s') or key == ord('S'):
                name = "{}\{}.jpg".format(path,frame_counter)
                cv2.imwrite(name, image_left)
                cv2.imshow("RealSense", image_left)
                frame_counter += 1
            if key == 27 or key==ord('q'): # ESC or 'q'
                stop=False

finally:
    pipe.stop()
