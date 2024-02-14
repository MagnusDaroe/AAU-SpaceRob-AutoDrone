import numpy as np
import matplotlib.pyplot as plt
import csv

def define_3d_circle(center, normal, radius, num_points=30):
    """
    Define a 3D circle.

    Args:
        center (numpy.ndarray): Center point of the circle (3D vector).
        normal (numpy.ndarray): Normal vector to the plane of the circle (3D vector).
        radius (float): Radius of the circle.
        num_points (int): Number of points to generate along the circumference.

    Returns:
        numpy.ndarray: Array of 3D points representing the circumference of the circle.
    """
    # Generate an array of angles to sweep around the circle
    angles = np.linspace(0, 2 * np.pi, num_points)

    # Calculate points on the circumference using parametric equations
    circle_points = center + radius * (np.cos(angles)[:, np.newaxis] * np.cross(normal, [1, 0, 0]) +
                                       np.sin(angles)[:, np.newaxis] * np.cross(normal, [0, 1, 0]))

    return circle_points

# Define parameters
center_point = np.array([0, 0, 0])  # Center point of the circle
radius = 40.0  # Radius of the circle
normal_vector = np.array([0, 0.2, 0.9]) / np.sqrt(3)  # Normal vector perpendicular to the plane of the circle

# Define the 3D circle
circle_points = define_3d_circle(center_point, normal_vector, radius)

# Calculate the relative orientation between each point
relative_orientations = []
for i in range(len(circle_points) - 1):
    # Calculate the direction vector between the two points
    direction = circle_points[i + 1] - circle_points[i]
    direction /= np.linalg.norm(direction)

    # Save the relative orientation with coordinates as one tuple
    relative_orientations.append((tuple(circle_points[i]), tuple(direction)))

# Print relative orientations with coordinates
print("Relative Orientations with Coordinates:")
print(relative_orientations)


# Plot the 3D circle with the same axis scales
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(circle_points[:, 0], circle_points[:, 1], circle_points[:, 2], 'b-')
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(-20, 20)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

# Specify the file path
csv_file_path = "trajectory.csv"

# Write the trajectory data to the CSV file
with open(csv_file_path, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["x", "y", "z", "roll", "pitch", "yaw"])  # Write header row
    for point in relative_orientations:
        position, euler = point
        writer.writerow(list(position) + list(euler))

