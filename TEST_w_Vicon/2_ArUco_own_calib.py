import pyrealsense2 as rs
from numpy.linalg import inv
#from scipy.spatial.transform import Rotation as Rot
import numpy as np
import cv2
import time




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
    
    def __init__(self):
        # Define the dictionary and the marker size
        ARUCO_DICT = cv2.aruco.DICT_5X5_100  # Dictionary ID
        #ARUCO_DICT = cv2.aruco.DICT_4X4_100

        #Marker size in m
        self.MARKER_SIZE=0.1515152 # meters

        self.dist=np.array([[-0.22814816,0.04330513,-0.00027584,0.00057192,-0.00322855]])
        self.mtx=np.array( [[289.17101938,0.,426.23687843],[0.,289.14205298,401.22256516],[0.,0.,1.]])
        


        self.dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

        self.marker_data={}

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()

        # Start streaming with requested configuration
        self.pipe.start()


    def get_marker_pose(self,image,output_image):
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
            r_vec,t_vec,_objPoints = cv2.aruco.estimatePoseSingleMarkers(marker_corners, self.MARKER_SIZE , self.mtx, self.dist)
            
            ##############################
            for i in range(len(r_vec)):
                cv2.aruco.drawDetectedMarkers(output_image, marker_corners, marker_ids)
                cv2.drawFrameAxes(output_image, self.mtx, self.dist, r_vec[i], t_vec[i], length=0.1, thickness=2) 
            ##############################
        return r_vec,t_vec,np.array(marker_corners),marker_ids

    def t_FCvicon_marker(self,t_vec_flat_meter):
        """Transform the t_vec from the camera frame to the vicon frame
        """
        y_left_Cam=32.000 #mm

        P_Cam_marker=np.array([[t_vec_flat_meter[2]*1000],[t_vec_flat_meter[0]*1000+y_left_Cam],[t_vec_flat_meter[1]*1000]])
        theta=np.deg2rad(75)
        roty_75=np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
        P_CamFlat_marker=roty_75@P_Cam_marker
        P_FC_Cam=np.array([[154.763],[-9.100],[92.357]]) #mm
        P_FC_marker=P_FC_Cam+P_CamFlat_marker
        t_FCvicon_marker_=np.array([-P_FC_marker[0][0],P_FC_marker[1][0],-P_FC_marker[2][0]])
        return t_FCvicon_marker_



       

    def get_undist_image_and_marker_pose(self,image_left,frames):
        image_rgb = cv2.cvtColor(image_left, cv2.COLOR_GRAY2RGB)


        r_vec,t_vec_,coner,Marker_ID=self.get_marker_pose(image_left,image_rgb)

        
        if r_vec is not None:
            for i in range(len(r_vec)):
                
                t_vec_flat=t_vec_[i].flatten() # meters

                t_FCvicon_marker_=self.t_FCvicon_marker(t_vec_flat)
                print("t_FCvicon_marker [mm] ",[round(t_FCvicon_marker_[0],3),round(t_FCvicon_marker_[1],3),round(t_FCvicon_marker_[2],3)]) #mm
                
                # if Marker_ID is in the marker_data dictionary update the dictionary with the new data
                    # else add the new data to the dictionary
                if f"ID_{Marker_ID[i][0]}" in self.marker_data:
                    self.marker_data.update({f"ID_{Marker_ID[i][0]}":np.array([round(t_vec_flat[0]*1000,3),round(t_vec_flat[1]*1000,3),round(t_vec_flat[2]*1000,3)])}) #mm
                else:
                    self.marker_data[f"ID_{Marker_ID[i][0]}"]=np.array([round(t_vec_flat[0]*1000,3),round(t_vec_flat[1]*1000,3),round(t_vec_flat[2]*1000,3)])
                
                #print the pos data on the image top left corner of the marker
                top_left_coner=coner[i][0][0].astype(int)
                cv2.putText(image_rgb, str("Cam to Marker [cm] x: {}, y: {}, z: {}".format(round(t_vec_flat[0]*100,3),round(t_vec_flat[1]*100,3),round(t_vec_flat[2]*100,3))),(20, 20), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
                cv2.putText(image_rgb, str("FC_Vicon to Marker [cm] x: {}, y: {}, z: {}".format(round(t_FCvicon_marker_[0]*0.1,3),round(t_FCvicon_marker_[1]*0.1,3),round(t_FCvicon_marker_[2]*0.1,3))),(20, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
        return image_rgb, self.marker_data


    #manin function
    def main(self):
        # Declare the variable stop to stop the while loop
        stop = True

        try:
            # The while loop will run until the variable stop is set to False by pressing the 'q' or 'ESC' key
            while stop==True:
                # Get the frames from the T265 camera, when the data is available
                frames = self.pipe.wait_for_frames()
                left_frame = frames.get_fisheye_frame(1)

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
                
                

        finally:
            cv2.destroyAllWindows()
            self.pipe.stop()


#create an instance of the T265 class
t265=T265()
#run the main function
t265.main()
