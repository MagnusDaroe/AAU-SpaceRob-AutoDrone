import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class DronePose:
    def __init__(self, x=None, y=None, z=None, w=None, qx=None, qy=None, qz=None, roll=None, pitch=None, yaw=None):
        """
        Initialize a DronePose object.

        The pose is saved as a position and an orientation described by a quaternion.
        
        Args:
            x (float): X-coordinate (position).
            y (float): Y-coordinate (position).
            z (float): Z-coordinate (position).
            w (float): Scalar part of the quaternion.
            qx (float): X-component of the quaternion vector.
            qy (float): Y-component of the quaternion vector.
            qz (float): Z-component of the quaternion vector.
            roll (float): Roll angle (in radians).
            pitch (float): Pitch angle (in radians).
            yaw (float): Yaw angle (in radians).
        """
        # Check if we have enough information to create a pose
        if x is not None and y is not None and z is not None:
            self.x = x
            self.y = y
            self.z = z
        if w is not None and qx is not None and qy is not None and qz is not None:
            self.w = w
            self.qx = qx
            self.qy = qy
            self.qz = qz
        elif roll is not None and pitch is not None and yaw is not None:
            self.w, self.qx, self.qy, self.qz = self.__eul2quat(roll, pitch, yaw)
        else:
            raise ValueError("Not enough information to create a pose.")

    def __str__(self):
        """
        Return a string representation of the DronePose.

        Returns:
            str: Formatted pose information.
        """
        return f"Position (x, y, z): ({self.x}, {self.y}, {self.z}), " \
               f"Orientation (w, qx, qy, qz): ({self.w}, {self.qx}, {self.qy}, {self.qz})"

    def __eul2quat(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.

        Args:
            roll (float): Roll angle (in radians).
            pitch (float): Pitch angle (in radians).
            yaw (float): Yaw angle (in radians).

        Returns:
            tuple: Quaternion components (w, qx, qy, qz).
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return w, qx, qy, qz

    def __quat2eul(self):
        """
        Convert Quaternion to Euler representation
        :return: Euler representation of the Quaternion
        """
        roll = np.arctan2(2 * (self.w * self.qx + self.qy * self.qz), 1 - 2 * (self.qx ** 2 + self.qy ** 2))
        pitch = np.arcsin(2 * (self.w * self.qy - self.qz * self.qx))
        yaw = np.arctan2(2 * (self.w * self.qz + self.qx * self.qy), 1 - 2 * (self.qy ** 2 + self.qz ** 2))

        return roll, pitch, yaw

    @property
    def quaternion(self):
        """Return the quaternion as a numpy array [w, qx, qy, qz]."""
        return np.array([self.w, self.qx, self.qy, self.qz])

def HamiltonProd(q1, q2):
    """
    The product of two quaternions - Also called the Hamilton product.
    The product is equal to the rotation using the second quaternion followed by the rotation with the first quaternion.
    
    This version is the reverse of the normal Hamilton product, as the first quaternion is applied first.

    :Note: The order of the quaternions is important when dealing with multiple axis rotations - when only one axis is rotated, the order does not matter.
    
    :param q1: Quaternion 1
    :param q2: Quaternion 2
    :return: The product of the two quaternions
    """
    # Unpack the quaternions
    w1, x1, y1, z1 = q2
    w2, x2, y2, z2 = q1

    # Calculate the Hamilton product
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return w, x, y, z

Trajectory = [((0.0, 20.784609690826528, -4.618802153517007), (-0.9938511382041374, -0.10808782498329239, 0.024019516662953974)), ((-4.468076694851296, 20.29867706647226, -4.510817125882724), (-0.9452765484378205, -0.3185007526974567, 0.07077794504387927)), ((-8.72723108936192, 18.863600862848926, -4.191911302855317), (-0.8512885035911208, -0.512203298057012, 0.11382295512378038)), ((-12.57830985775467, 16.546483648265323, -3.676996366281183), (-0.7176634887856662, -0.6798069662518207, 0.15106821472262702)), ((-15.841240836986136, 13.45567124838456, -2.9901491663076794), (-0.5519286866141979, -0.8140339111037115, 0.18089642468971348)), ((-18.363453000954767, 9.735686615833801, -2.163485914629734), (-0.3624944981487109, -0.9097925868587402, 0.2021761304130534)), ((-20.027010512110873, 5.560472097565162, -1.2356604661255917), (-0.15802684163842282, -0.9639210908823495, 0.21420468686274433)), ((-20.754127270144167, 1.1252560840352424, -0.2500569075633872), (0.05285334718145034, -0.9748226289284274, 0.21662725087298385)), ((-20.51080410358125, -3.362575653351858, 0.7472390340781907), (0.261598614631124, -0.9421930572163523, 0.20937623493696716)), ((-19.308418533256322, -7.693177090422641, 1.709594908982809), (0.459633274596775, -0.8669603253257502, 0.19265785007238906)), ((-17.20319277208221, -11.664054117097471, 2.592012026021661), (0.6382959443481309, -0.7514600790283262, 0.16699112867296131)), ((-14.293564836861021, -15.089532936921879, 3.3532295415381954), (0.7889903075174773, -0.5997748387924334, 0.1332832975094296)), ((-10.71558569602452, -17.80944196742713, 3.957653770539362), (0.903650124963883, -0.4180730668460251, 0.09290512596578336)), ((-6.636557677560024, -19.696601285308567, 4.377022507846348), (0.9755080921812364, -0.21472556847131208, 0.047716792993624864)), ((-2.247211598096915, -20.662769418288992, 4.5917265373975535), (1.0, 0.0, 0.0)), ((2.2472115980969103, -20.662769418288992, 4.5917265373975535), (0.9755080921812364, 0.21472556847131172, -0.04771679299362478)), ((6.6365576775600275, -19.696601285308567, 4.377022507846348), (0.9036501249638829, 0.41807306684602485, -0.09290512596578342)), ((10.715585696024526, -17.80944196742713, 3.9576537705393617), (0.788990307517477, 0.5997748387924335, -0.13328329750942966)), ((14.293564836861027, -15.089532936921877, 3.3532295415381945), (0.6382959443481303, 0.7514600790283267, -0.16699112867296137)), ((17.203192772082208, -11.664054117097475, 2.5920120260216613), (0.4596332745967752, 0.86696032532575, -0.19265785007238895)), ((19.308418533256322, -7.693177090422641, 1.709594908982809), (0.261598614631124, 0.9421930572163523, -0.20937623493696714)), ((20.51080410358125, -3.3625756533518585, 0.7472390340781908), (0.05285334718145034, 0.9748226289284274, -0.21662725087298385)), ((20.754127270144167, 1.125256084035242, -0.25005690756338705), (-0.15802684163842343, 0.9639210908823492, -0.2142046868627443)), ((20.02701051211087, 5.5604720975651665, -1.2356604661255925), (-0.36249449814871015, 0.9097925868587405, -0.20217613041305335)), ((18.363453000954767, 9.735686615833806, -2.1634859146297343), (-0.5519286866141985, 0.8140339111037113, -0.18089642468971362)), ((15.841240836986133, 13.455671248384565, -2.9901491663076807), (-0.7176634887856661, 0.679806966251821, -0.151068214722627)), ((12.578309857754666, 16.54648364826533, -3.6769963662811844), (-0.8512885035911208, 0.5122032980570118, -0.11382295512378039)), ((8.727231089361927, 18.863600862848926, -4.191911302855317), (-0.9452765484378205, 0.31850075269745665, -0.07077794504387926)), ((4.468076694851303, 20.29867706647226, -4.510817125882724), (-0.9938511382041372, 0.10808782498329235, -0.024019516662953964))]

# Instantiate the DronePose objects, based on the trajectory
DronePoses = []
for i in range(len(Trajectory)):
     print(*Trajectory[i][0],*Trajectory[i][1])
     DronePoses.append((DronePose(*Trajectory[i][0],None,None,None,None,*Trajectory[i][1])))

# plot the droneposes in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(len(DronePoses)):
    ax.scatter(DronePoses[i].x, DronePoses[i].y, DronePoses[i].z, c='r', marker='o')
    #ax.text(DronePoses[i].x, DronePoses[i].y, DronePoses[i].z, f'{i}', color='black')
    #Unpack the quaternion and convert it to euler angles
    roll, pitch, yaw = DronePoses[i]._DronePose__quat2eul()
    ax.quiver(DronePoses[i].x, DronePoses[i].y, DronePoses[i].z, roll, pitch, yaw, length=1, normalize=True)
    
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(-20, 20)
plt.show()

