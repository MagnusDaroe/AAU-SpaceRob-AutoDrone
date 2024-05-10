"""
opencv-contrib-python         4.7.0.72
opencv-python                 4.8.1.78
"""

import pyrealsense2 as rs
#from numpy.linalg import inv
import numpy as np
from numpy.linalg import inv
import cv2
import time
import math
from scipy.spatial.transform import Rotation as R


class T265():
    
    def __init__(self):
        ##########
        self.t_old=0
        ##########

        self.T_global_start=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.R_global_start=self.T_global_start[:3,:3]

        self.P_CAM_FC_local=np.array([-153.223-01.54,9.1,(-05.75-86.6070)])
        print("P_CAM_FC_local: ",self.P_CAM_FC_local)
        time.sleep(1)
        self.T_start_ref=np.array([[1,0,0,-self.P_CAM_FC_local[0]],[0,1,0,-self.P_CAM_FC_local[1]],[0,0,1,-self.P_CAM_FC_local[2]],[0,0,0,1]])
        self.R_start_ref=self.T_start_ref[:3,:3]

        self.T_global_cam=self.T_global_start@self.T_start_ref
        self.R_global_cam=self.T_global_cam[:3,:3]
        
        
        # transformation from cam to pose frame center in T265 rotation of -75 degrees around the y-axis:
        angle=-75
        self.T_ref_cam=np.array([[np.cos(np.deg2rad(angle)),0,np.sin(np.deg2rad(angle)),0],
                                    [0,1,0,0],
                                    [-np.sin(np.deg2rad(angle)),0,np.cos(np.deg2rad(angle)),0],
                                    [0,0,0,1]])
        self.R_ref_cam=self.T_ref_cam[:3,:3]



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
            self.translation_xyz_mm = [data.translation.x*1000, data.translation.y*1000, data.translation.z*1000] #mm
            self.rotation_xyzw = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
            self.pose_confidence = data.tracker_confidence


    def q_to_RPY(self,qrot_xyzw):
        w = qrot_xyzw[3]
        x = -qrot_xyzw[2]
        y = qrot_xyzw[0]
        z = -qrot_xyzw[1]

        self.pitch =  -math.asin(2.0 * (x*z - w*y)) #* 180.0 / math.pi
        self.roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) #* 180.0 / math.pi
        self.yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) #* 180.0 / math.pi

        ###########
        #self.pitch+=np.deg2rad(75)
        ###########
        #return roll, pitch, yaw

    def RPY_to_T(self):
        """Convert excentric roll, pitch, and yaw to a rotation matrix and then Transformation matrix
        """
        # Calculate trigonometric values
        cos_r = np.cos(self.roll)
        sin_r = np.sin(self.roll)
        cos_p = np.cos(self.pitch)
        sin_p = np.sin(self.pitch)
        cos_y = np.cos(self.yaw)
        sin_y = np.sin(self.yaw)
        
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
        
        R_mtx_local= R_z @ R_y @ R_x
        R_mtx=inv(self.R_ref_cam@inv(R_mtx_local))
        self.P_CAM_FC=np.dot(R_mtx,self.P_CAM_FC_local)
        T_mtx=np.array([[R_mtx[0,0], R_mtx[0,1], R_mtx[0,2], -1*self.translation_xyz_mm[2]+self.P_CAM_FC[0]],
                    [R_mtx[1,0], R_mtx[1,1], R_mtx[1,2], self.translation_xyz_mm[0]+self.P_CAM_FC[1]],
                    [R_mtx[2,0], R_mtx[2,1], R_mtx[2,2], -1*self.translation_xyz_mm[1]+self.P_CAM_FC[2]],
                    [0, 0, 0, 1]])
        return T_mtx

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

    def R_to_euler_angles_V2(self):
        """use scipy to convert a rotation matrix to euler angles
        """
        
        r = R.from_matrix(self.r_mtx_global)
        self.euler_xyz=r.as_euler('xyz', degrees=False)

    def get_global_pose(self):
        """Get the global pose of the camera
        """
        self.T_cam_FC=self.RPY_to_T()
        self.T_global_FC=self.T_global_cam@self.T_cam_FC
        
        #t_vec_cam_FC=self.T_cam_FC[:3,3]
        #t_vec_global_cam=np.array([self.T_global_cam[0][3],self.T_global_cam[1][3],self.T_global_cam[2][3]])
        #self.t_vec_global_FC=t_vec_global_cam+t_vec_cam_FC
        self.t_vec_global_FC=np.array([self.T_global_FC[0][3],self.T_global_FC[1][3],self.T_global_FC[2][3]])
        
        self.R_global_FC=self.T_global_FC[:3,:3]

        
        self.r_mtx_global=self.R_global_FC
        

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
            
            self.q_to_RPY(self.rotation_xyzw)

            print("RPY * [deg]:[ {0:.7f}, {1:.7f}, {2:.7f}]".format(np.rad2deg(self.roll), np.rad2deg(self.pitch), np.rad2deg(self.yaw)))
            self.get_global_pose()
            print("Global pose: x: {}, y: {}, z: {}".format(round(self.t_vec_global_FC[0]*0.1,2),round(self.t_vec_global_FC[1]*0.1,2),round(self.t_vec_global_FC[2]*0.1,2)))
            
            self.R_to_euler_angles_V2()
            #print("Euler angles xyz: ",self.euler_xyz)
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

