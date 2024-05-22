import numpy as np
# [RX,RY,RZ,TX,TY,TZ,t_image_marker] where RX,RY and RZ are in degrees, TX,TY and TZ are in mm, and t_image_marker is the position of the marker in the image plane in mm
test_1=[0.79, -1.95, -0.22, -234.6, 1288.6, 866.25, [-24.576, -272.991, 869.896]]
test_2=[0.75, -2.93, -24.76, -272.95, 993.34, 866.65, [-1.836, -314.069, 876.631]]
test_3=[0.9, -2.44, 30.32, -416.83, 1546.39, 863.6, [-13.838, -181.344, 832.138]]
test_4=[1.54, -1.932, 2.178, 126.98, 1385.9, 1579.9, [-127.656, -483.907, 1726.001]]
test_5=[-0.29, -3.09, -88.19, -779.6, 640.289, 490.52, [-53.425, -382.485, 505.548]]
test_6=[-0.15, -3.21, -97.01, -1005.57, 829.38, 489.54, [104.871, -215.212, 462.607]]
test_7=[3.93, 3.6, 188.2, -1165.88, 1200.23, 489.76, [-17.939, -82.562, 415.907]]
test_8=[0.412, -2.19, -37.95, -196.69, 772.876, 863.57, [-22.715, -487.583, 940.771]]
test_9=[-0.188, -2.099, -73.017, -615.675, 619.32, 862.2, [-43.369, -356.115, 897.817]]
#test_10=[1.837, -1.59, 49.41, -49.6, 1839.69, 864.9, [-75.661, -352.599, 892.698]] #aflæsningsfejl på TX coordinatet skrev RZ ind i stedet for TX
test_11=[1.28, -1.685, 31.46, -213.95, 1532.81, 865.06, [109.377, -340.53, 898.029]]

P_VICON_MARKER_REAL=np.array([[-842.3],[1269.8],[0]]) #mm

all_tests=[test_1,test_2,test_3,test_4,test_5,test_6,test_7,test_8,test_9,test_11]


def P_Vicon_FCvicon(roll,pitch,yaw,t_image_marker):
    """Compute the position of the marker in the Vicon frame
    """
    y_cam_img_diff=32.000 #mm
    P_CamFalt_marker=np.array([[t_image_marker[2]], [t_image_marker[0]+y_cam_img_diff],   [t_image_marker[1]]])
    theta=np.deg2rad(-75)
    roty_75=np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    P_Cam_marker=roty_75@P_CamFalt_marker

    x_FC_Cam_diff=154.763 #mm
    y_FC_Cam_diff=-9.100 #mm
    z_FC_Cam_diff=92.357 #mm
    P_FC_marker=P_Cam_marker+np.array([[x_FC_Cam_diff],[y_FC_Cam_diff],[z_FC_Cam_diff]])

    theta_2=np.deg2rad(180)
    roty_180=np.array([[np.cos(theta_2),0,np.sin(theta_2)],[0,1,0],[-np.sin(theta_2),0,np.cos(theta_2)]])
    P_FCvicon_marker=roty_180@P_FC_marker

    cos_r = np.cos(roll)
    sin_r = np.sin(roll)
    cos_p = np.cos(pitch)
    sin_p = np.sin(pitch)
    cos_y = np.cos(yaw)
    sin_y = np.sin(yaw)
    
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
    
    #Rotation matrix from the Vicon frame to the FC_vicon frame:
    R_vicon_FCvicon=R_z@R_y@R_x
    P_vicon_marker=np.array([[TX],[TY],[TZ]])+R_vicon_FCvicon@P_FCvicon_marker

    return P_vicon_marker

P_vicon_marker_list=[]
marker_diff_list=[]
marker_discance_diff_list=[]
distance_marker_drone_liset=[]
for test in all_tests:
    RX=test[0]
    RY=test[1]
    RZ=test[2]
    TX=test[3]
    TY=test[4]
    TZ=test[5]
    t_image_marker=test[6]
    P_vicon_marker=P_Vicon_FCvicon(np.deg2rad(RX),np.deg2rad(RY),np.deg2rad(RZ),t_image_marker)
    #print("P_vicon_marker [mm] x: {}, y: {}, z: {}".format(round(P_vicon_marker[0][0],2),round(P_vicon_marker[1][0],2),round(P_vicon_marker[2][0],2))) #mm

    p_marker_diff=P_vicon_marker-P_VICON_MARKER_REAL
    print("p_marker_diff [mm] x: {}, y: {}, z: {}".format(round(p_marker_diff[0][0],2),round(p_marker_diff[1][0],2),round(p_marker_diff[2][0],2))) #mm
    marker_discance_diff=np.sqrt(p_marker_diff[0][0]*p_marker_diff[0][0]+p_marker_diff[1][0]*p_marker_diff[1][0]+p_marker_diff[2][0]*p_marker_diff[2][0])#np.linalg.norm(p_marker_diff)
    print("marker_discance_diff [mm] ",round(marker_discance_diff,2)) #mm
    
    marker_diff_list.append([p_marker_diff[0][0],p_marker_diff[1][0],p_marker_diff[2][0]])
    P_vicon_marker_list.append([P_vicon_marker[0][0],P_vicon_marker[1][0],P_vicon_marker[2][0]])
    marker_discance_diff_list.append(marker_discance_diff)
    diff_drone_marker=P_vicon_marker-np.array([TX,TY,TZ])
    distance_marker_drone_liset.append(np.sqrt(diff_drone_marker[0][0]*diff_drone_marker[0][0]+diff_drone_marker[1][0]*diff_drone_marker[1][0]+diff_drone_marker[2][0]*diff_drone_marker[2][0]))



#plot the marker_discance_diff as a function of P_vicon_marker z coordinate with bares
import matplotlib.pyplot as plt
fig, ax = plt.subplots()
ax.bar(distance_marker_drone_liset, marker_discance_diff_list)
plt.xlabel('P_vicon_marker z coordinate [mm]')
plt.ylabel('marker_discance_diff [mm]')
plt.title('marker_discance_diff as a function of P_vicon_marker z coordinate')
plt.show()

#save the distance_marker_drone, marker_diff in x,y,z, and marker_discance_diff to a csv file called ArUco_marker_diff.csv
import csv
with open('ArUco_marker_diff.csv', mode='w') as file:
    writer = csv.writer(file)
    writer.writerow(['distance_marker_drone','marker_diff_x','marker_diff_y','marker_diff_z','marker_discance_diff'])
    for i in range(len(distance_marker_drone_liset)):
        writer.writerow([distance_marker_drone_liset[i],marker_diff_list[i][0],marker_diff_list[i][1],marker_diff_list[i][2],marker_discance_diff_list[i]])
