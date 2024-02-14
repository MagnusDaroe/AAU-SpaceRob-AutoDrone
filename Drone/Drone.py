import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import random as rnd

class DronePose:
    def __init__(self, x=None, y=None, z=None, w=None, qx=None, qy=None, qz=None, roll=None, pitch=None, yaw=None, t = None):
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
            t: time in seconds. 
        """
        # Check if we have enough information to create a pose
        if x is not None and y is not None and z is not None and t is not None:
            self.x = x
            self.y = y
            self.z = z
            self.t = t
            if w is not None and qx is not None and qy is not None and qz is not None:
                self.w = w
                self.qx = qx
                self.qy = qy
                self.qz = qz
            elif roll is not None and pitch is not None and yaw is not None:
                self.w, self.qx, self.qy, self.qz = self.__eul2quat(roll, pitch, yaw)
            else:
                raise ValueError("Not enough information to create a pose.")
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

    @property
    def coordinates(self):
        """Return the coordinates as a numpy array [w, qx, qy, qz]."""
        return np.array([self.x, self.y, self.z])
    @property
    def quaternion(self):
        """Return the quaternion as a numpy array [w, qx, qy, qz]."""
        return np.array([self.w, self.qx, self.qy, self.qz])
    
    @property
    def quaternion_inv(self):
        """Return the inverse of the quaternion as a numpy array [w, qx, qy, qz]."""
        return np.array([self.w, -self.qx, -self.qy, -self.qz])
    @property
    def euler(self):
        """
        Convert Quaternion to Euler representation
        :return: Euler representation of the Quaternion
        """
        roll = np.arctan2(2 * (self.w * self.qx + self.qy * self.qz), 1 - 2 * (self.qx ** 2 + self.qy ** 2))
        pitch = np.arcsin(2 * (self.w * self.qy - self.qz * self.qx))
        yaw = np.arctan2(2 * (self.w * self.qz + self.qx * self.qy), 1 - 2 * (self.qy ** 2 + self.qz ** 2))
        return np.array([roll, pitch, yaw])

def HamiltonProd(q1, q2):
    """
    The product of two quaternions - Also called the Hamilton product.
    The product is equal to the rotation using the second quaternion followed by the rotation with the first quaternion.
    
    This version is the reverse of the normal Hamilton product, as the first quaternion is applied first.

    :Note: The order of the quaternions is important when dealing with multiple axis rotations - when only one axis is rotated, the order does not matter.
    
    :param q1: Quaternion 1
    :param q2: Quaternion 2
    :return: The product of the two quaternions

    Based on wikipedia: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Hamilton_product
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

def ReadTrajec(path):
    # Read the trajectory data from the CSV file
    Trajectory = []
    with open(path, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip the header row
        for row in reader:
            # Convert the row values to the appropriate data types
            Trajectory.append(row[:6])
    return Trajectory

def Add_sensornoise(Trajectory):
    Trajectory_temp = []
    # Maximal error in decimal percentage
    Max_error = 0.20
    Accumulative_error = np.zeros(6)  # Initialize as a NumPy array
    for i in range(len(Trajectory)):
        # Accumulate the error based on randomness
        noise = np.random.uniform(0, Max_error, size=6)
        Accumulative_error += noise
        # Add the error to the trajectory
        noisy_data = np.array(Trajectory[i], dtype=float) + Accumulative_error
        Trajectory_temp.append(noisy_data)
    return Trajectory_temp

# Get the poses for a given trajectory
Trajectory = ReadTrajec("trajectory.csv")

# Instantiate the DronePose objects, based on the trajectory - Normally given by measurements. 
DronePoses = []
for i in range(len(Trajectory)):
    DronePoses.append(DronePose(float(Trajectory[i][0]), float(Trajectory[i][1]), float(Trajectory[i][2]), roll= float(Trajectory[i][3]), pitch = float(Trajectory[i][4]), yaw = float(Trajectory[i][5]), t = 0))

"""
relative_orientations = []
for i, pose in enumerate(reversed(DronePoses[:-1]), start=1):
    # Compute the relative orientation between the current pose and the previous pose
    relative_orientation = HamiltonProd(DronePoses[-i].quaternion_inv, pose.quaternion)

    # Add the relative orientation to the list
    relative_orientations.append(DronePose(*pose.coordinates,None, None, None, None, *relative_orientation, 0))
"""

Trajectory_noise = Add_sensornoise(Trajectory)

DronePoses_n = []
for i in range(len(Trajectory_noise)):
    DronePoses_n.append(DronePose(float(Trajectory_noise[i][0]), float(Trajectory_noise[i][1]), float(Trajectory_noise[i][2]), None, None, None, None, float(Trajectory_noise[i][3]), float(Trajectory_noise[i][4]), float(Trajectory_noise[i][5]), 0))


DronePoses_n[1]

# plot the droneposes in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(len(DronePoses)):
    if i == 0:
        ax.text(*DronePoses[i].coordinates, f'{i}', color='black')
    ax.scatter(*DronePoses[i].coordinates, c='r', marker='o')
    ax.scatter(*DronePoses[i].coordinates, c='g', marker='o')
    #ax.text(DronePoses[i].x, DronePoses[i].y, DronePoses[i].z, f'{i}', color='black')
    #Unpack the quaternion and convert it to euler angles
    roll, pitch, yaw = DronePoses[i].euler
    ax.quiver(DronePoses[i].x, DronePoses[i].y, DronePoses[i].z, roll, pitch, yaw, length=1, normalize=True)
    ax.quiver(DronePoses_n[i].x, DronePoses_n[i].y, DronePoses_n[i].z, roll, pitch, yaw, length=1, normalize=True)
    
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(-20, 20)
plt.show()

