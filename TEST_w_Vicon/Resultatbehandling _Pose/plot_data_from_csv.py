"""load data from 10sec_data.csv and plot the discance and diff yaw for each of the 5 test
"""
import matplotlib.pyplot as plt
import numpy as np
# Load the data from the 10sec_data.csv file
data = np.loadtxt("10sec_data.csv", delimiter=",")
time = data[:,0]
diff_distance = data[:,4]
diff_yaw = data[:,5]

# Plot the diff_distance as a function of time in seconds as pillars
plt.figure()
plt.bar(time, diff_distance, label="Diff in distance")
plt.xlabel("Time (s)")
plt.ylabel("Diff in distance (mm)")
plt.legend()
plt.grid()
plt.show()

# Plot the diff_yaw as a function of time in seconds as pillars
plt.figure()
plt.bar(time, diff_yaw, label="Diff in yaw")
plt.xlabel("Time (s)")
plt.ylabel("Diff in yaw (degrees)")
plt.legend()
plt.grid()
plt.show()


#plot the diff_distance and diff_yaw as pillars
