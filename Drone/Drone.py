import numpy as np
import matplotlib.pyplot as plt

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

Pose2d = np.array([(45,0,0),(45,2,2),(0,4,4),(0,6,4),(-45,8,4),(45,10,2),(90,12,4),(135,12,6),(180,10,8),(180,8,8)])

Pose2d_relative = np.array([(45,0,0),(0,2,2),(-45,4,4),(0,6,4),(-45,8,4),(90,10,2),(45,12,4),(45,12,6),(45,10,8),(0,8,8)])

# plot the trajectory based on the relative poses
Poses1 = []
for i, pose_values in enumerate(Pose2d_relative):
    # Unpack quat values from the pose
    yaw, x, y = pose_values
    if i == 0:
        Poses1.append(DronePose(x=x, y=y, z=0, roll=0, pitch=0, yaw=np.radians(yaw)))
    else:
        # Calculate the relative quaternion
        q_relative = HamiltonProd(Poses1[i-1].quaternion, DronePose(x=x, y=y, z=0, roll=0, pitch=0, yaw=np.radians(yaw)).quaternion)
        Poses1.append(DronePose(x=x, y=y, z=0, w=q_relative[0], qx=q_relative[1], qy=q_relative[2], qz=q_relative[3]))

Poses2 = []
for pose_values in Pose2d:
    yaw, x, y = pose_values
    Poses2.append(DronePose(x=x, y=y, z=0, roll=0, pitch=0, yaw=np.radians(yaw)))

# plot the path based on the poses given as objects of the class. Where x and y are the position of the drone and the yaw is the orientation shown with the arrow
fig, ax = plt.subplots()
for pose in Poses1:
    # get the euler angles from the quaternion
    roll, pitch, yaw = pose._DronePose__quat2eul()
    ax.quiver(pose.x, pose.y, np.cos(yaw), np.sin(yaw))
    ax.plot(pose.x, pose.y, 'ro')
plt.show()  




