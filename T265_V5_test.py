"""
opencv-contrib-python         4.7.0.72
opencv-python                 4.8.1.78
"""

import pyrealsense2 as rs
#from numpy.linalg import inv
import numpy as np
import cv2
import time
from scipy.spatial.transform import Rotation
import math


class T265():
    
    def __init__(self):
        ##########
        self.o=0
        self.t_old=0
        self.Xframe_number=0
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
        ###############
        A_vec=np.array([[2],[3],[-4]])
        B_vec=self.R_local_cam@A_vec
        print("B_vec: ",B_vec)
        ###############


        __CALIBRATION_FILE = "_Calibrate_camera_2\calib_new_1.npz"

        with np.load(__CALIBRATION_FILE) as X:
            self.ret,self.mtx, self.dist = [X[i] for i in ("ret","mtx", "dist")]


        # Define the dictionary and the marker size
        ARUCO_DICT=cv2.aruco.DICT_5X5_100

        #Marker size in cm
        self._MARKER_SIZE=10

        self.dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

        
        self.marker_data={}

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        # Build configuation object and request pose data from the T265
        self.cfg = rs.config()
        # Enable the pose stream
        self.cfg.enable_stream(rs.stream.pose)

        # Start streaming with requested configuration
        self.pipe.start(self.cfg)
        # Start streaming with requested configuration
        #self.pipe.start()

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
    """
    def get_YPY_V1(self,rotation_xyzw):
        #Convert a quaternion to a rotation matrix
        
        
        #w = rotation_xyzw[3]
        #x = rotation_xyzw[0]
        #y = rotation_xyzw[1]
        #z = rotation_xyzw[2]
        
        w =rotation_xyzw[3]
        x =-rotation_xyzw[2]
        y =rotation_xyzw[0]
        z =-rotation_xyzw[1]

        self.pitch =  -np.arcsin(2.0 * (x*z - w*y)) * 180.0 / np.pi
        self.roll  =  np.arctan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / np.pi
        self.yaw   =  np.arctan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / np.pi
    
    def get_YPY_V2(self,R_mtx_global):
        #Convert r_mtx_global to pitch, roll and yaw
        
        #pitch
        self.pitch = np.arctan2(R_mtx_global[2,1],R_mtx_global[2,2])
        #roll
        self.roll = np.arctan2(-R_mtx_global[2,0],np.sqrt(R_mtx_global[2,1]**2+R_mtx_global[2,2]**2))
        #yaw
        self.yaw = np.arctan2(R_mtx_global[1,0],R_mtx_global[0,0])

    def get_YPY_V3(self,R_mtx_global):
        #Convert r_mtx_global to pitch, roll and yaw
        
        sy = math.sqrt(R_mtx_global[0,0] * R_mtx_global[0,0] +  R_mtx_global[1,0] * R_mtx_global[1,0])
 
        singular = sy < 1e-6
 
        if  not singular :
            x = math.atan2(R_mtx_global[2,1] , R_mtx_global[2,2])
            y = math.atan2(-R_mtx_global[2,0], sy)
            z = math.atan2(R_mtx_global[1,0], R_mtx_global[0,0])
        else :
            x = math.atan2(-R_mtx_global[1,2], R_mtx_global[1,1])
            y = math.atan2(-R_mtx_global[2,0], sy)
            z = 0
 
        return np.array([x, y, z])
    """
    def R_to_euler_angles(self):
        """Convert a rotation matrix to euler angles
        """
        r = Rotation.from_matrix(self.r_mtx_global)
        self.euler_xyz = r.as_euler('xyz', degrees=True)
        self.euler_zyx = r.as_euler('zyx', degrees=True)


    def get_global_pose(self):
        """Get the global pose of the camera
        """
        T_q=self.__get_T_matrix_q(self.rotation_xyzw,self.translation_xyz)

        R_rot=T_q[:3,:3]
        
        self.t_vec_global=self.T_global_cam@np.array([[self.translation_xyz[0]*100],[self.translation_xyz[1]*100],[self.translation_xyz[2]*100],[1]])
        self.r_mtx_global=self.R_global_cam@R_rot
        

    def update_start_frame(self,T_global_UAV):
        """Update the start frame of the camera
        """
        self.T_global_cam=T_global_UAV
        self.R_global_cam=self.T_global_cam[:3,:3]
    


    def _get_marker_pose(self):
        """Get the pose data of the markers in the image
        """
        t_vec=None
        r_vec=None

        marker_corners, marker_ids, rejectedCandidates = self.detector.detectMarkers(self.img_undist)

        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in marker_corners:
            cv2.cornerSubPix(self.img_undist, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

        # If at least one marker is detected get the pose data of the markers and draw the axis on the image
        if marker_ids is not None and len(marker_ids) > 0:
            r_vec,t_vec,_objPoints = cv2.aruco.estimatePoseSingleMarkers(marker_corners, self._MARKER_SIZE , self.mtx, self.dist)
        
        self.M_r_vec=r_vec
        self.M_t_vec=t_vec
        self.M_corners=np.array(marker_corners)
        self.M_IDs=marker_ids



    def get_pose_and_marker_pos(self,image_left):
        self.img_undist = cv2.undistort(src = image_left, cameraMatrix = self.mtx, distCoeffs = self.dist)
        self.image_rgb = cv2.cvtColor(self.img_undist, cv2.COLOR_GRAY2RGB)

        #get the pose data from the T265 camera
        self.get_pose_data(self.frames) #mabye not needed

        T_q=self.__get_T_matrix_q(self.rotation_xyzw,self.translation_xyz)
        
        self._get_marker_pose()
        
        if self.M_r_vec is not None:
            for i in range(len(self.M_r_vec)):
                
                t_vec_flat=self.M_t_vec[i].flatten()

                P_center=self.__T_C_LEFT@np.array([[t_vec_flat[0]],[t_vec_flat[1]],[t_vec_flat[2]],[1]])
                
                P_center_1x3=np.array([P_center[0],P_center[1],P_center[2],[-1]])

                P_marker_pos=T_q@(P_center_1x3)
                
                P_marker_pos=P_marker_pos.round(0)
                
                #Get the center xy coordinat of the marker
                center_x=int((self.M_corners[i][0][0][0]+self.M_corners[i][0][2][0])/2)
                center_y=int((self.M_corners[i][0][0][1]+self.M_corners[i][0][2][1])/2)

            
                # if self.M_IDs is in the marker_data dictionary update the dictionary with the new data
                    # else add the new data to the dictionary
                if f"ID_{self.M_IDs[i][0]}" in self.marker_data:
                    self.marker_data.update({f"ID_{self.M_IDs[i][0]}":[P_marker_pos.flatten()[:-1],[center_x,center_y]]})
                else:
                    self.marker_data[f"ID_{self.M_IDs[i][0]}"]=[P_marker_pos.flatten()[:-1],[center_x,center_y]]
                

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

            self.get_pose_and_marker_pos(self.image_left)
            
            
            
            
            
            #print the pose data from the T265 camera and the marker ID and position
            print("Marker Data:")
            for key, value in self.marker_data.items():
                print(f"{key}: {value}")
            #time.sleep(0.1)
            self.get_global_pose()
            #print("Global pose: x: {}, y: {}, z: {}".format(round(self.t_vec_global[0][0],2),round(self.t_vec_global[1][0],2),round(self.t_vec_global[2][0],2)))
            self.R_to_euler_angles()
            #print("Euler angles xyz: ",self.euler_xyz)
            t=time.time()
            print("time diff in ms: ",(t-self.t_old)*1000)
            print("time stamp: ",self.time_stamp)
            self.t_old=t
            left_frame=None
            if left_frame:
                print("left_frame is not None")
            self.pipe.stop()
            #self.pipe = rs.pipeline()
            #self.cfg = rs.config()
            # Enable the pose stream
            #self.cfg.enable_stream(rs.stream.pose)
            self.pipe.start(self.cfg)

            self.Xframe_number=0
        else:
            #get the pose data from the T265 camera
            self.get_pose_data(self.frames)
            #get time in ms
            t=time.time()
            self.get_global_pose()
            print("time diff in ms: ",(t-self.t_old)*1000)
            print("time stamp: ",self.time_stamp)
            print("Global pose: x: {}, y: {}, z: {}".format(round(self.t_vec_global[0][0],2),round(self.t_vec_global[1][0],2),round(self.t_vec_global[2][0],2)))
            self.R_to_euler_angles()
            print("Euler angles xyz: ",self.euler_xyz)
            self.t_old=t
            self.Xframe_number+=1
        
        if self.Xframe_number<5000:
            self.pipe.stop()
            #self.pipe = rs.pipeline()
            self.pipe.start()

           

        #print("\n\nT265 Pose Data\nPosition: x: {}, y: {}, z: {}".format(round(self.translation_xyz[0]*100,2),round(self.translation_xyz[1]*100,2),round(self.translation_xyz[2]*100,2)))
        #print("Rotation: x: {}, y: {}, z: {}, w: {}".format(self.rotation_xyzw[0],self.rotation_xyzw[1],self.rotation_xyzw[2],self.rotation_xyzw[3]))
        #self.get_global_pose()
        #print("Global pose: x: {}, y: {}, z: {}".format(round(self.t_vec_global[0][0],2),round(self.t_vec_global[1][0],2),round(self.t_vec_global[2][0],2)))
        #self.R_to_euler_angles()
        #print("Euler angles xyz: ",self.euler_xyz)



#create an instance of the T265 class
t265=T265()



#run the main function
#t265.main()
try:
    while t265.stop==False:
        t265.run()
        #t265.show_image()
finally:
    cv2.destroyAllWindows() 
    t265.pipe.stop()

