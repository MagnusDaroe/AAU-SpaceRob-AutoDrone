#!/usr/bin/env python3
import pyrealsense2 as rs
from numpy.linalg import inv
import numpy as np
import cv2
import time
import math
import rclpy
from rclpy.node import Node
import threading
from drone.msg import DroneControlData, ViconData

class RateController:
    def __init__(self, rate):
        self.rate = rate
        self.start_time = time.time()

    def sleep(self):
        elapsed_time = time.time() - self.start_time
        sleep_time = max(0, 1 / self.rate - elapsed_time)
        time.sleep(sleep_time)
        self.start_time = time.time()

class T265(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Flag to indicate if the global frame has been updated
        self.global_frame_updated = False

        # Initialize the needed math
        self.math_init()

        # Initialize the camera
        self.camera_init()

        # Create publisher and subscriber
        self.publisher_ = self.create_publisher(DroneControlData, '/DroneControlData', 10)

        self.subscriber_ = self.create_subscription(ViconData, '/ViconData', self.update_global_pos, 10)

    def math_init(self):
        #self.R_FC_backside=np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.T_FC_backside=np.array([[1,0,0,153.223],[0,1,0,0],[0,0,1,86.6070],[0,0,0,1]]) #mm

        # Transformation from backside center of T265 to cam frame center in T265:
        self.T_backside_center=np.array([[1,0,0,1.54],[0,1,0,-9.10],[0,0,1,05.75],[0,0,0,1]]) #mm

        # Transformation from pose frame to the FC:
        self.T_pose_FC=inv(self.T_FC_backside@self.T_backside_center)

        # Transformation from the pose frame to the poseCam frame, where the camera is rotated -75 degrees around the y-axis:
        angle=-75
        self.T_pose_poseCam=np.array([[np.cos(np.deg2rad(angle)),0,np.sin(np.deg2rad(angle)),0],
                                    [0,1,0,0],
                                    [-np.sin(np.deg2rad(angle)),0,np.cos(np.deg2rad(angle)),0],
                                    [0,0,0,1]])
        self.R_pose_poseCam=self.T_pose_poseCam[:3,:3]

        __P_CAM_FC_local=np.array([self.T_pose_FC[0,3],self.T_pose_FC[1,3],self.T_pose_FC[2,3]])
        self.T_start_ref=np.array([[1,0,0,-1*__P_CAM_FC_local[0]],[0,1,0,-1*__P_CAM_FC_local[1]],[0,0,1,-1*__P_CAM_FC_local[2]],[0,0,0,1]])

        self.T_Vicon_drone_start=np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
        self.T_global_vicon=np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])
        self.T_global_start=self.T_global_vicon@self.T_Vicon_drone_start


        self.T_global_ref=self.T_global_start@self.T_start_ref

        self.dist=np.array([[-0.22814816,0.04330513,-0.00027584,0.00057192,-0.00322855]])
        self.mtx=np.array( [[289.17101938,0.,426.23687843],[0.,289.14205298,401.22256516],[0.,0.,1.]])
        self.diff_x=0.0
        self.diff_y=0.0
        self.diff_z=0.0

        self.vicon_x=0
        self.vicon_y=0
        self.vicon_z=0


    def camera_init(self):
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()

        cfg = rs.config()

        cfg.enable_stream(rs.stream.pose)
        # Start streaming with requested configuration
        self.pipe.start(cfg)

        # variable to stop and start camera loop
        self.stop=False

   
    def update_global_pos(self, msg):
        """Update the global position of the drone
        """
        if not self.global_frame_updated:
            roll=msg.vicon_roll
            pitch=msg.vicon_pitch
            yaw=msg.vicon_yaw
            x=msg.vicon_x
            y=msg.vicon_y
            z=msg.vicon_z
            #Extrinsic rotation matrix from roll, pitch and yaw
            T_global=np.array([[np.cos(yaw)*np.cos(pitch),np.cos(yaw)*np.sin(pitch)*np.sin(roll)-np.sin(yaw)*np.cos(roll),np.cos(yaw)*np.sin(pitch)*np.cos(roll)+np.sin(yaw)*np.sin(roll),x],
                            [np.sin(yaw)*np.cos(pitch),np.sin(yaw)*np.sin(pitch)*np.sin(roll)+np.cos(yaw)*np.cos(roll),np.sin(yaw)*np.sin(pitch)*np.cos(roll)-np.cos(yaw)*np.sin(roll),y],
                            [-np.sin(pitch),np.cos(pitch)*np.sin(roll),np.cos(pitch)*np.cos(roll),z],
                            [0,0,0,1]])
            
            self.update_start_frame(T_global)
            
            self.frames = self.pipe.wait_for_frames()
            left_frame = self.frames.get_fisheye_frame(1)

            # If the frame is available, get the image from the left camera and show it undistorted in a window
            if left_frame:          
                self.image_left = np.asanyarray(left_frame.get_data())
                self.get_T265_pose_data(self.frames)


                self.q_to_RPY()
                self.get_global_pose()

            self.global_frame_updated = True
        else:
            P_global=[msg.vicon_x,msg.vicon_y,msg.vicon_z]
            self.vicon_x=msg.vicon_x
            self.vicon_y=msg.vicon_y
            self.vicon_z=msg.vicon_z
            #self.update_position(P_global)
            
        

    def get_T265_pose_data(self,frames):
        """Get the pose data from the T265 camera
        """
        # Get the data from the latest frame in the pipeline
        pose = frames.get_pose_frame()
        data = pose.get_pose_data()
        if pose:
            #self.time_stamp=pose.timestamp
            self.translation_xyz_mm = [data.translation.x*1000, data.translation.y*1000, data.translation.z*1000] #mm
            self.rotation_xyzw = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
            #self.pose_confidence = data.tracker_confidence

    def q_to_RPY(self):
        """
        Convert a quaternion to roll pitch and yaw angles, 
        NB: the pitch is -75 degrees off due to the camera orientation

        Surce: https://github.com/IntelRealSense/librealsense/issues/5178#issuecomment-550217609
        """
        qrot_xyzw=self.rotation_xyzw
        w = qrot_xyzw[3]
        x = -qrot_xyzw[2]
        y = qrot_xyzw[0]
        z = -qrot_xyzw[1]

        self.pitch =  -math.asin(2.0 * (x*z - w*y)) #rad - NB: the pitch is -75 degrees off
        self.roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) #rad
        self.yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) #rad

    def RPY_and_pos_to_T(self):
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
        
        # Compute rotation of the cam frame to poseCam frame
        R_cam_poseCam= R_z @ R_y @ R_x

        # Compute the rotation matrix from the pose frame to the poseCam frame
        R_mtx=R_cam_poseCam@inv(self.R_pose_poseCam)

        # Compute the Transformation matrix of the translation from the ref frame to the cam frame:
        T_ref_cam=np.array([[1,0,0, -1*self.translation_xyz_mm[2]],
                    [0,1,0, self.translation_xyz_mm[0]],
                    [0,0,1, -1*self.translation_xyz_mm[1]],
                    [0, 0, 0, 1]])
        
        # Compute the Transformation matrix of the rotation from the ref frame to the cam frame:
        T_cam_pose=np.array([[R_mtx[0,0], R_mtx[0,1], R_mtx[0,2], 0],
                    [R_mtx[1,0], R_mtx[1,1], R_mtx[1,2], 0],
                    [R_mtx[2,0], R_mtx[2,1], R_mtx[2,2], 0],
                    [0, 0, 0, 1]])
        
        # Compute the Transformation matrix from the ref frame to the pose frame:
        T_ref_pose=T_ref_cam@T_cam_pose
        ######################## HUSK at flytte T_pose_FC ############################

        return T_ref_pose




    def get_global_pose(self):
        """Get the global pose of the camera
        """
        # Convert the quaternion and translation to a trnasformation matrix from ref frame to pose frame
        self.T_ref_pose=self.RPY_and_pos_to_T()
        # Compute the transformation from global frame to FC frame
        self.T_global_FC=self.T_global_ref@self.T_ref_pose@self.T_pose_FC
     
        # Get the global position of the camera
        self.t_vec_global_FC=np.array([self.T_global_FC[0][3]+self.diff_x,self.T_global_FC[1][3]+self.diff_y,self.T_global_FC[2][3]+self.diff_z])
        
        # Get the global rotation of the camera
        self.r_mtx_global=self.T_global_FC[:3,:3]

    def R_to_euler_angles(self):
        """Convert a rotation matrix to euler angles
        """
        #get extrinsic euler angles
            #sqrt(R11^2+R21^2)
        sy = math.sqrt(self.r_mtx_global[0,0] * self.r_mtx_global[0,0] +  self.r_mtx_global[1,0] * self.r_mtx_global[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(self.r_mtx_global[2,1] , self.r_mtx_global[2,2]) #atan2(R32,R33)
            y = math.atan2(-self.r_mtx_global[2,0], sy) #atan2(-R31,sqrt(R11^2+R21^2))
            z = math.atan2(self.r_mtx_global[1,0], self.r_mtx_global[0,0]) #atan2(R21,R11)
        else :
            x = math.atan2(-self.r_mtx_global[1,2], self.r_mtx_global[1,1]) #atan2(-R23,R22)
            y = math.atan2(-self.r_mtx_global[2,0], sy) #atan2(-R31,sqrt(R11^2+R21^2))
            z = 0
        
        ################new stuff:################
                # Adjust angles to be between -2*pi and 2*pi
        x = x % (2 * math.pi)
        y = y % (2 * math.pi)
        z = z % (2 * math.pi)

        # Convert angles < -pi to positive equivalents
        if x < -math.pi:
            x += 2 * math.pi
        if y < -math.pi:
            y += 2 * math.pi
        if z < -math.pi:
            z += 2 * math.pi

        ##############################
        self.euler_xyz=[x,y,z]
        
        
        

    def update_start_frame(self,T_vicon_start):
        """Update the start frame of the camera
        """
        self.T_global_start=self.T_global_vicon@T_vicon_start@self.T_Vicon_drone_start
        self.T_global_ref=self.T_global_start@self.T_start_ref
        self.P_vicon_start=np.array([T_vicon_start[0,3],T_vicon_start[1,3],T_vicon_start[2,3]])

    def update_position(self,P_vicon_FC):
        """Update the global position of the drone
        """
        self.diff_x=(-1*P_vicon_FC[0])-self.T_global_FC[0,3] #mm
        self.diff_y=(-1*P_vicon_FC[1])-self.T_global_FC[1,3] #mm
        self.diff_z=P_vicon_FC[2]-self.T_global_FC[2,3] #mm
        

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
        
        #Create rate controller
        rate_controller = RateController(100)
        msg = DroneControlData()

        while rclpy.ok():
            if self.global_frame_updated:
                taketime=time.time()
                # Get the frames from the T265 camera, when the data is available
                
                self.frames = self.pipe.wait_for_frames()
               
                left_frame = self.frames.get_fisheye_frame(1)


                # If the frame is available, get the image from the left camera and show it undistorted in a window
                if left_frame:          
                    self.image_left = np.asanyarray(left_frame.get_data())
                    self.get_T265_pose_data(self.frames)


                    self.q_to_RPY()
                    self.get_global_pose()
                    #self.get_logger().info(f"cam pose: x: {round(self.translation_xyz_mm[0],2)}, y: {round(self.translation_xyz_mm[1],2)}, z: {round(self.translation_xyz_mm[2],2)}")
                    #log the diff_x, diff_y and diff_z
                    #self.get_logger().info(f"diff_x: {round(self.diff_x,2)}, diff_y: {round(self.diff_y,2)}, diff_z: {round(self.diff_z,2)}")

                    #self.get_logger().info(f"vicon pose: x: {round(self.vicon_x,2)}, y: {round(self.vicon_y,2)}, z: {round(self.vicon_z,2)}")
                    ##self.get_logger().info(f"Global pose: x: {round(self.t_vec_global_FC[0],2)}, y: {round(self.t_vec_global_FC[1],2)}, z: {round(self.t_vec_global_FC[2],2)}")
                    self.R_to_euler_angles()
                    self.update_position([self.vicon_x,self.vicon_y,self.vicon_z])
                    #self.get_logger().info(f"Euler angles xyz: {self.euler_xyz}")
                    ##self.get_logger().info(f"Euler angles xyz deg: x: {round(math.degrees(self.euler_xyz[0]),2)}, y: {round(math.degrees(self.euler_xyz[1]),2)}, z: {round(math.degrees(self.euler_xyz[2]),2)}")
                
                    self.get_logger().info(f"Time to get frame: {time.time()-taketime}")
                msg.timestamp = time.time()
                msg.camera_x = float(self.t_vec_global_FC[0]) # mm
                msg.camera_y = float(self.t_vec_global_FC[1]) # mm
                msg.camera_z = float(self.t_vec_global_FC[2]) # mm
                #msg.camera_pitch = float(self.euler_xyz[0]) # rad
                #msg.camera_roll = float(self.euler_xyz[1]) # rad
                msg.camera_yaw = float(self.euler_xyz[2]) # rad
                self.publisher_.publish(msg)
                
            else: 
                self.get_logger().warning('No global frame data available')
                time.sleep(0.2)
            
            #rate_controller.sleep()'

            

def main(args=None):
    rclpy.init(args=args)
    #create an instance of the T265 class
    camera=T265()

    #make a thread for the camera loop
    threading.Thread(target=camera.run, daemon=True).start()

    camera.get_logger().info('camera node has been started :-)')
    rclpy.spin(camera)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
