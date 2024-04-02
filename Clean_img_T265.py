"""
This code is have been written and modifyed based on the code from wrappers/python/examples/t265_stereo.py from the librealsense library:
https://github.com/IntelRealSense/librealsense/commit/b6ee94d4d19d5a6223139d40e92e1378449cb2d7

The code shows how to get the image from the T265 camera with out fisheye distortion.
"""

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2
from math import tan, pi


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

def undistort_rectify_func(intrinsics):
    """
    Returns the undistort_rectify map and the max disparity from the left camera
    """

    # Set the disparity parameters
    min_disp = 0
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp

    # Translate the intrinsics from librealsense into OpenCV
    K_left  = camera_matrix(intrinsics)
    D_left  = fisheye_distortion(intrinsics)
    #(width, height) = (intrinsics.width, intrinsics.height)
    
    # We need to determine what focal length our undistorted images should have
    # in order to set up the camera matrices for initUndistortRectifyMap.  We
    # could use stereoRectify, but here we show how to derive these projection
    # matrices from the calibration and a desired height and field of view

    # We calculate the undistorted focal length:
    #
    #         h
    # -----------------
    #  \      |      /
    #    \    | f  /
    #     \   |   /
    #      \ fov /
    #        \|/
    stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
    stereo_height_px = 300         # 300x300 pixel stereo output
    stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

    # The stereo algorithm needs max_disp extra pixels in order to produce valid
    # disparity on the desired output region. This changes the width, but the
    # center of projection should be on the center of the cropped image
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2

    # The left rotation is set to identity
    R_left = np.eye(3)

    # Construct the left projection matrices
    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                    [0, stereo_focal_px, stereo_cy, 0],
                    [0,               0,         1, 0]])
    
    # Create an undistortion map for the left camera which applies the
    # rectification and undoes the camera distortion. This only has to be done
    # once
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    undistort_rectify = (lm1, lm2)
    return undistort_rectify, max_disp
    

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Start streaming with requested configuration
pipe.start()

# Declare the variable stop to stop the while loop
stop = True

try:
    # Get the intrinsics of the left camera from the T265 camera and calculate the undistort_rectify map and the max disparity
    intrinsics = pipe.get_active_profile().get_stream(rs.stream.fisheye, 1).as_video_stream_profile().get_intrinsics()
    undistort_rectify, max_disp = undistort_rectify_func(intrinsics)


    # The while loop will run until the variable stop is set to False by pressing the 'q' or 'ESC' key
    while stop==True:
        # Get the frames from the T265 camera, when the data is available
        frames = pipe.wait_for_frames()
        left_frame = frames.get_fisheye_frame(1)

        # If the frame is available, get the image from the left camera and show it undistorted in a window
        if left_frame:
            image_left = np.asanyarray(left_frame.get_data())

            # Undistort the image from the left camera using the undistort_rectify map
            center_undistorted = {"left" : cv2.remap(src = image_left,
                                map1 = undistort_rectify[0],
                                map2 = undistort_rectify[1],
                                interpolation = cv2.INTER_LINEAR)}


            cv2.namedWindow('RealSense_flat', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense_flat',center_undistorted["left"][:,max_disp:])

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', image_left)

            
            key = cv2.waitKey(1)
            if key == 27 or key==ord('q'): # ESC or 'q'
                stop=False
    
finally:
    pipe.stop()
