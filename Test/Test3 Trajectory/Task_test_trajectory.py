import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x_ref_list = np.array([1016, 1016, 1420, 358, -1082, -1780, -1362,-87,-110,-528,1211])
y_ref_list = np.array([-970, -970,370, 1439, 1158, 488, -997, -1494, 33, 1031, -92])
z_ref_list = np.array([0, 500, 1000,1200,900,600,1100,1500,1800,1300,900])


points = np.vstack((x_ref_list, y_ref_list, z_ref_list)).T

# Plot the line
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot all the points
ax.scatter(points[:,0], points[:,1], points[:,2], c='red', marker='o')

# Make lines between the points
for i in range(len(points)-1):
    x = [points[i][0], points[i+1][0]]
    y = [points[i][1], points[i+1][1]]
    z = [points[i][2], points[i+1][2]]
    ax.plot(x, y, z)

unit_graph="mm"
# Set x, y, z range
ax.set_xlim([-2000, 2000])
ax.set_ylim([-2000, 2000])
ax.set_zlim([0, 2000])
ax.set_xlabel(f'X axis [{unit_graph}]')
ax.set_ylabel(f'Y axis [{unit_graph}]')
ax.set_zlabel(f'Z axis [{unit_graph}]')


plt.show()
