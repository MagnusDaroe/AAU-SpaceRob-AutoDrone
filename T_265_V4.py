"""
opencv-contrib-python         4.7.0.72
opencv-python                 4.8.1.78
"""

import pyrealsense2 as rs
#from numpy.linalg import inv
import numpy as np
import cv2
import time
import math


class T265():
    
    def __init__(self):
        ##########
        self.t_old=0
        ##########

        __LEFT_2_C_MM=-32.00 #Distance from the left camera sensor to the center of the T265 in mm
        __LENS_2_C_MM=6.55 #Distance from the lense to the center of the T265 in mm

        __LEFT_2_C_CM=__LEFT_2_C_MM/10 #Distance from the left camera sensor to the center of the T265 in cm
        __LENS_2_C_CM=__LENS_2_C_MM/10 #Distance from the lense to the center of the T265 in cm

        __THETA=-np.pi
        # Define the Transformation matrix from the left camera sensor to pose center of the T265
        self.__T_C_LEFT=np.array([[1 , 0,0,__LEFT_2_C_CM],
                        [0,np.cos(__THETA),-np.sin(__THETA),0],
                        [0,np.sin(__THETA),np.cos(__THETA),__LENS_2_C_CM],
                        [0,0,0,1]])
        
        self.R_local_cam=np.array([[0,0,-1],[-1,0,0],[0,1,0]]) 
        self.T_local_cam=np.array([[0,0,-1,0],[-1,0,0,0],[0,1,0,0],[0,0,0,1]])
        
        self.R_global_cam=self.R_local_cam
        self.T_global_cam=self.T_local_cam


        __CALIBRATION_FILE = "_Calibrate_camera_2\calib_new_1.npz"

        with np.load(__CALIBRATION_FILE) as X:
            self.ret,self.mtx, self.dist = [X[i] for i in ("ret","mtx", "dist")]

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()

        # Start streaming with requested configuration
        self.pipe.start()

        # variable to stop and start camera loop
        self.stop=False

    def get_pose_data(self,frames):
        """Get the pose data from the T265 camera
        """
        # Get the data from the latest frame in the pipeline
        pose = frames.get_pose_frame()
        data = pose.get_pose_data()
        if pose:
            self.time_stamp=pose.timestamp
            self.translation_xyz = [data.translation.x, data.translation.y, data.translation.z]
            self.rotation_xyzw = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
            self.pose_confidence = data.tracker_confidence

    def __get_T_matrix_q(self,qrot_xyzw,translation_xyz):
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

    def R_to_euler_angles(self):
        """Convert a rotation matrix to euler angles
        """
        #get extrinsic euler angles
        sy = math.sqrt(self.r_mtx_global[0,0] * self.r_mtx_global[0,0] +  self.r_mtx_global[1,0] * self.r_mtx_global[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(self.r_mtx_global[2,1] , self.r_mtx_global[2,2])
            y = math.atan2(-self.r_mtx_global[2,0], sy)
            z = math.atan2(self.r_mtx_global[1,0], self.r_mtx_global[0,0])
        else :
            x = math.atan2(-self.r_mtx_global[1,2], self.r_mtx_global[1,1])
            y = math.atan2(-self.r_mtx_global[2,0], sy)
            z = 0
        self.euler_xyz=[x,y,z]


    def get_global_pose(self):
        """Get the global pose of the camera
        """
        T_q=self.__get_T_matrix_q(self.rotation_xyzw,self.translation_xyz)

        R_rot=T_q[:3,:3]
        
        self.t_vec_global=self.T_global_cam@np.array([[self.translation_xyz[0]*100],[self.translation_xyz[1]*100],[self.translation_xyz[2]*100],[1]])
        self.r_mtx_global=self.R_global_cam@R_rot
        print(self.r_mtx_global)
        

    def update_start_frame(self,T_global_UAV):
        """Update the start frame of the camera
        """
        self.T_global_cam=T_global_UAV
        self.R_global_cam=self.T_global_cam[:3,:3]


    def show_image(self,undist=True):
        """
        Show undistorted image 
        If q or ESC is pressed the loop will stop
        """
        if undist:
            undist_image = cv2.undistort(src = self.image_left, cameraMatrix = self.mtx, distCoeffs = self.dist)
            cv2.imshow('Undist_image',undist_image)
        else:
            cv2.imshow('Image_rgb',self.image_left)
        key = cv2.waitKey(1)
        if key == 27 or key==ord('q'): # ESC or 'q'
            self.stop=True
            return True
        else:
            self.stop=False
            return False
            
    
    def run(self):
        # Get the frames from the T265 camera, when the data is available
        self.frames = self.pipe.wait_for_frames()
        left_frame = self.frames.get_fisheye_frame(1)

        # If the frame is available, get the image from the left camera and show it undistorted in a window
        if left_frame:          
            self.image_left = np.asanyarray(left_frame.get_data())
            self.get_pose_data(self.frames)
            
            t=time.time()
            print("time diff in ms: ",(t-self.t_old)*1000)
            self.t_old=t


            self.get_global_pose()
            print("Global pose: x: {}, y: {}, z: {}".format(round(self.t_vec_global[0][0],2),round(self.t_vec_global[1][0],2),round(self.t_vec_global[2][0],2)))
            self.R_to_euler_angles()
            print("Euler angles xyz: ",self.euler_xyz)
            print("Euler angles xyz deg: x: {}, y: {}, z: {}".format(round(math.degrees(self.euler_xyz[0]),2),round(math.degrees(self.euler_xyz[1]),2),round(math.degrees(self.euler_xyz[2]),2)))
        time.sleep(0.1)


#create an instance of the T265 class
t265=T265()



#run the main function
#t265.main()
try:
    while t265.stop==False:
        t265.run()
        t265.show_image()
finally:
    cv2.destroyAllWindows() 
    t265.pipe.stop()

