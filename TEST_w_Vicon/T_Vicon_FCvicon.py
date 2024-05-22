import numpy as np
def T_Vicon_FCvicon(roll,pitch,yaw,t_flat,Extrinsic=True):
    """Transform mation matrix from the Vicon frame to the FC_vcion frame
    """
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
    if Extrinsic==True:
        R_vicon_FCvicon=R_z@R_y@R_x
    else:
        R_vicon_FCvicon=R_x@R_y@R_z
    
    #The transformation matrix from the Vicon frame to the FC_vicon frame:
    T_vicon_FCvicon=np.array([[R_vicon_FCvicon[0][0],R_vicon_FCvicon[0][1],R_vicon_FCvicon[0][2],t[0]],
                            [R_vicon_FCvicon[1][0],R_vicon_FCvicon[1][1],R_vicon_FCvicon[1][2],t[1]],
                            [R_vicon_FCvicon[2][0],R_vicon_FCvicon[2][1],R_vicon_FCvicon[2][2],t[2]],
                            [0,0,0,1]])
    return T_vicon_FCvicon
"""
roll=0 #rad
pitch=0 #rad
yaw=0 #rad
t=[0,0,0] #mm

roll=0.01 #rad
pitch=-0.04 #rad
yaw=0.06 #rad
t=[-1091, 572, 1592]

Marker=[-465.34, -95.109, -1680.851] #mm

roll=np.deg2rad(-0.29) #rad
pitch=np.deg2rad(-3.09) #rad
yaw=np.deg2rad(-88.19) #rad
t=[-779.6, 640.289, 490.52]

Marker=[-66.603, 127.51, 410.422] #mm
"""


roll=np.deg2rad(1.28) #rad
pitch=np.deg2rad(-1.685) #rad
yaw=np.deg2rad(31.46) #rad
t=[-213.95, 1532.81, 865.06]

Marker=[-58.472, 132.176, 861.348] #mm


T_Vicon_FCvicon_=T_Vicon_FCvicon(roll,pitch,yaw,t,Extrinsic=True)

P_FCvicon_marker=np.array([[Marker[0]],[Marker[1]],[Marker[2]],[1]])

P_Vicon_marker=T_Vicon_FCvicon_@P_FCvicon_marker

print("P_Vicon_marker [mm] - x: {}, y: {}, z: {}".format(P_Vicon_marker[0][0],P_Vicon_marker[1][0],P_Vicon_marker[2][0])) #mm

print("P_Vicon_marker list [mm]: ",P_Vicon_marker.flatten()[:3]) #mm