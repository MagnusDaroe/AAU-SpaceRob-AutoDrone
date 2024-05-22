import numpy as np
import matplotlib.pyplot as plt

# Load data from the CSV file

_, _, _, _, dist = np.loadtxt(r"Pose st\ArUco_marker_diff.csv", delimiter=',', unpack=True, skiprows=1)

# Round the data


# Define the bins for the histogram (example: 10s intervals)


mean_all = np.mean(dist)
std_all = np.std(dist)
print(f"Mean: {mean_all:.2f}, Standard Deviation: {std_all:.2f}")