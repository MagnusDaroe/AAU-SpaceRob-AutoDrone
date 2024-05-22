import numpy as np
"""
"""
### Test 1 ###
roll=0.01 #rad
pitch=-0.04 #rad
yaw=0.06 #rad
t=[-1091, 572, 1592]

Marker=[-354.255, 1588.494, -2.695] #mm
"""
### Test 2 ###
roll=0.03 #rad
pitch=0.01 #rad
yaw=1.44 #rad
t=[-1671, 1080, 620] #mm

Marker=[-297.569, 512.895, -6.578] #mm
P_Vicon_marker=[-2218.277463,851.96412527,631.78470189]

### Test 3 ###
roll=0.0 #rad
pitch=0.03 #rad
yaw=2.91 #rad
t=[-1999, 784, 617] #mm

Marker=[-247.188, 519.373, -31.172] #mm
P_Vicon_marker=[-1876.82022619,221.56754565,593.25655405]

### Test 4 ###
roll=0.0 #rad
pitch=-0.02 #rad
yaw=-0.04 #rad
t=[-1087, 645, 880] #mm
Marker=[-301.041, 844.492, 147.896] #mm
P_Vicon_marker=[-1356.92472542,  1500.97079539,  1021.84600317]
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


def t_FCvicon_marker(t_vec_flat_mm):
    """Transform the t_vec from the camera frame to the vicon frame
    """
    y_left_Cam=32.000 #mm

    P_Cam_marker=np.array([[t_vec_flat_mm[2]],[t_vec_flat_mm[0]+y_left_Cam],[t_vec_flat_mm[1]]])
    theta=np.deg2rad(75)
    rotz_75=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
    P_CamFlat_marker=rotz_75@P_Cam_marker
    P_FC_Cam=np.array([[154.763],[-9.100],[92.357]]) #mm
    P_FC_marker=P_FC_Cam+P_CamFlat_marker
    t_FCvicon_marker_=np.array([-P_FC_marker[0][0],P_FC_marker[1][0],-P_FC_marker[2][0]])
    return t_FCvicon_marker_

t_FCvicon_marker_=t_FCvicon_marker(np.array(Marker))

print("t_FCvicon_marker [mm] ",[round(t_FCvicon_marker_[0],3),round(t_FCvicon_marker_[1],3),round(t_FCvicon_marker_[2],3)]) #mm

