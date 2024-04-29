"""
opencv-contrib-python         4.7.0.72
opencv-python                 4.8.1.78
"""

import pyrealsense2 as rs
from numpy.linalg import inv
#from scipy.spatial.transform import Rotation as Rot
import numpy as np
import cv2
import time

# Define the dictionary and the marker size
ARUCO_DICT = cv2.aruco.DICT_5X5_100  # Dictionary ID

#Marker size in cm
MARKER_SIZE=10

#calibration file location for the T265 camera
CALIBRATION_FILE = "_Calibrate_camera_2\calib_new_1.npz"

class T265():

    __LEFT_2_C_MM=-32.00 #Distance from the left camera sensor to the center of the T265 in mm
    __LENS_2_C_MM=6.55 #Distance from the lense to the center of the T265 in mm

    __LEFT_2_C_CM=__LEFT_2_C_MM/10 #Distance from the left camera sensor to the center of the T265 in cm
    __LENS_2_C_CM=__LENS_2_C_MM/10 #Distance from the lense to the center of the T265 in cm

    __THETA=-np.pi
    # Define the Transformation matrix from the left camera sensor to pose center of the T265
    T_C_LEFT=np.array([[1 , 0,0,__LEFT_2_C_CM],
                    [0,np.cos(__THETA),-np.sin(__THETA),0],
                    [0,np.sin(__THETA),np.cos(__THETA),__LENS_2_C_CM],
                    [0,0,0,1]])
    
    def __init__(self,call_file=CALIBRATION_FILE,aruco_dict=cv2.aruco.DICT_5X5_100,marker_size=10):

        with np.load(call_file) as X:
            self.ret,self.mtx, self.dist = [X[i] for i in ("ret","mtx", "dist")]

        
        self.MARKER_SIZE=marker_size


        self.dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

        self.marker_data={}

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()

        # Start streaming with requested configuration
        self.pipe.start()

    def get_T_matrix_q(self,qrot_xyzw,translation_xyz):
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

    def get_pose_data(self,frames):
        """Get the pose data from the T265 camera
        """
        # Get the data from the latest frame in the pipeline
        pose = frames.get_pose_frame()
        data = pose.get_pose_data()
        if pose:
            time_stamp=pose.timestamp
            frame_number = pose.frame_number
            translation_xyz = [data.translation.x, data.translation.y, data.translation.z]
            rotation_xyzw = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
            pose_confidence = data.tracker_confidence
        return translation_xyz, rotation_xyzw,time_stamp#, pose_confidence, frame_number, time_stamp

    def get_marker_pose(self,image,output_image,marker_size=10):
        """Get the pose data of the markers in the image
        """
        t_vec=[]
        r_vec=None

        marker_corners, marker_ids, rejectedCandidates = self.detector.detectMarkers(image)

        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in marker_corners:
            cv2.cornerSubPix(image, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

        # If at least one marker is detected get the pose data of the markers and draw the axis on the image
        if marker_ids is not None and len(marker_ids) > 0:
            r_vec,t_vec,_objPoints = cv2.aruco.estimatePoseSingleMarkers(marker_corners, marker_size , self.mtx, self.dist)
            for i in range(len(r_vec)):
                cv2.aruco.drawDetectedMarkers(output_image, marker_corners, marker_ids)
                cv2.drawFrameAxes(output_image, self.mtx, self.dist, r_vec[i], t_vec[i], length=10, thickness=2) 
        return r_vec,t_vec,np.array(marker_corners),marker_ids


    def get_undist_image_and_marker_pose(self,image_left,frames):
        img_undist = cv2.undistort(src = image_left, cameraMatrix = self.mtx, distCoeffs = self.dist)
        image_rgb = cv2.cvtColor(img_undist, cv2.COLOR_GRAY2RGB)

        #get the pose data from the T265 camera
        translation_xyz, rotation_xyzw,time_stamp = self.get_pose_data(frames)

        #print the pose data on the image top left corner
        pos_cm=np.array(translation_xyz)*100
        pos_cm=pos_cm.round(0).astype(int)
        cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(pos_cm[0],pos_cm[1],pos_cm[2])),(20, 20), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)

        T_q=self.get_T_matrix_q(rotation_xyzw,translation_xyz)
        
        r_vec,t_vec_,coner,Marker_ID=self.get_marker_pose(img_undist,image_rgb,self.MARKER_SIZE)

        
        if r_vec is not None:
            for i in range(len(r_vec)):
                
                t_vec_flat=t_vec_[i].flatten()

                P_center=self.T_C_LEFT@np.array([[t_vec_flat[0]],[t_vec_flat[1]],[t_vec_flat[2]],[1]])
                
                P_center_1x3=np.array([P_center[0],P_center[1],P_center[2],[-1]])

                P_marker_pos=T_q@(P_center_1x3)
                
                P_marker_pos=P_marker_pos.round(0)
                
                # if Marker_ID is in the marker_data dictionary update the dictionary with the new data
                    # else add the new data to the dictionary
                if f"ID_{Marker_ID[i][0]}" in self.marker_data:
                    self.marker_data.update({f"ID_{Marker_ID[i][0]}":P_marker_pos.flatten()[:-1]})
                else:
                    self.marker_data[f"ID_{Marker_ID[i][0]}"]=P_marker_pos.flatten()[:-1]
                
                #print the pos data on the image top left corner of the marker
                top_left_coner=coner[i][0][0].astype(int)
                cv2.putText(image_rgb, str("x: {}, y: {}, z: {}".format(P_marker_pos[0][0],P_marker_pos[1][0],P_marker_pos[2][0])),(top_left_coner[0], top_left_coner[1] - 20), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
        return image_rgb, self.marker_data


    #manin function
    def main(self):
        # Declare the variable stop to stop the while loop
        stop = True


        time_old=0
        hej=0
        test=0
        t_old=0
        try:
            # The while loop will run until the variable stop is set to False by pressing the 'q' or 'ESC' key
            while stop==True:
                # Get the frames from the T265 camera, when the data is available
                frames = self.pipe.wait_for_frames()
                left_frame = frames.get_fisheye_frame(1)

                #get the pose data from the T265 camera
                translation_xyz, rotation_xyzw,time_stamp = self.get_pose_data(frames)


                #get time in ms
                t=time.time()
                #print("time diff: ",time_stamp-time_old)
                print("time diff in ms: ",(t-t_old)*1000)
                t_old=t
                time_old=time_stamp
                
                print("\n\nT265 Pose Data\nPosition: x: {}, y: {}, z: {}".format(round(translation_xyz[0]*100,2),round(translation_xyz[1]*100,2),round(translation_xyz[2]*100,2)))
                print("Rotation: x: {}, y: {}, z: {}, w: {}".format(rotation_xyzw[0],rotation_xyzw[1],rotation_xyzw[2],rotation_xyzw[3]))
                
                # If the frame is available, get the image from the left camera and show it undistorted in a window
                if left_frame:          
                    image_left = np.asanyarray(left_frame.get_data())

                    image_rgb, marker_data=self.get_undist_image_and_marker_pose(image_left,frames)

                    image_rgb_sz=image_rgb[300:-300,300:-300]
                    
                    
                    
                    #print the pose data from the T265 camera and the marker ID and position
                    print("Marker Data:")
                    for key, value in marker_data.items():
                        print(f"{key}: {value}")
                    
                    
                    cv2.imshow('Image_with_with_detections',image_rgb_sz)
                    cv2.imshow('Image_rgb',image_rgb)
                    key = cv2.waitKey(1)
                    if key == 27 or key==ord('q'): # ESC or 'q'
                        stop=False
                else:
                    hej+=1
                    #print("No new Marker Data")
                
                

        finally:
            print(hej)
            cv2.destroyAllWindows()
            self.pipe.stop()


    #if __name__ == "__main__":
    #    main()

#create an instance of the T265 class
t265=T265()
#run the main function
t265.main()



