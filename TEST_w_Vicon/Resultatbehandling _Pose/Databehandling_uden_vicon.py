"""
Load data from "TEST_w_Vicon\Databehandling_uden_vicon.py"
The first column is the timestamp in millie seconds
The second column is the diff in x position in mm
The third column is the diff in y position in mm
The fourth column is the diff in z position in mm

plot the diff in x, y, and z position in mm as a function of time in seconds starting from 0
"""
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

#make a list of the files from the folder TEST_w_Vicon\Test_files
import os
path = r"TEST_w_Vicon\Test_files"
files = os.listdir(path)
print(files)
camera_data_diskret = files[0:10]
camera_data_uden_vicon=files[10:]
print("camera_data_diskret",camera_data_diskret)
print("camera_data_uden_vicon",camera_data_uden_vicon)

# Load the data from the csv file
#data = pd.read_csv(r"TEST_w_Vicon\Test_files\camera_data_uden_vicon_1.csv", header=None)
#data = pd.read_csv(r"TEST_w_Vicon\Test_files\camera_data_diskret_8.csv", header=None)

# Load the data from the csv file in camera_data_uden_vicon
start=True
i=0
for file in camera_data_uden_vicon:
    if start:
        #make len(camera_data_uden_vicon)x4 array
        data_at_10_sec_intival = np.zeros((len(camera_data_uden_vicon)*10,6))
        start=False
    path_to_file = os.path.join(path, file)
    data = pd.read_csv(path_to_file, header=None)
    # Get the timestamp in milliseconds
    time = data[0].values

    # Get the diff in x, y, and z position in mm
    diff_x = data[1].values
    diff_y = data[2].values
    diff_z = data[3].values
    roll_cam = data[4].values
    pitch_cam = data[5].values
    yaw_cam = data[6].values
    roll_vicon = data[7].values
    pitch_vicon = data[8].values
    yaw_vicon = data[9].values
    print("yaw_cam",np.rad2deg(yaw_cam[1000]))
    print("yaw_vicon",np.rad2deg(yaw_vicon[1000]))

    # Convert the timestamp to seconds
    time = (time - time[0]) / 1000
    time_points = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100] #s
    # for time = 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 seconds get the x,y,z diff in mm and wirte it to a csv file called 10sec_data.csv where the first column is time in seconds and the second, third, and fourth column is the diff in x, y, and z position in mm
    
    for t in time_points:
        idx = np.argmin(np.abs(time - t))
        print(f"Time: {t} s")
        print(f"Timestamp: {data[0].values[idx]}")
        print(f"Diff in x position: {diff_x[idx]} mm")
        print(f"Diff in y position: {diff_y[idx]} mm")
        print(f"Diff in z position: {diff_z[idx]} mm")
        diff_yaw_deg = np.rad2deg(yaw_cam[idx]-yaw_vicon[idx])
        print(f"diff in yaw in : {diff_yaw_deg} degrees")
        #disctance
        print(f"Distance from the origin: {np.sqrt(diff_x[idx]**2 + diff_y[idx]**2 + diff_z[idx]**2)}")
        print()
        data_i_sec = np.array([time[idx], diff_x[idx], diff_y[idx], diff_z[idx],np.sqrt(diff_x[idx]**2 + diff_y[idx]**2 + diff_z[idx]**2),diff_yaw_deg])
        #append the data to data_at_10_sec_intival
        
        data_at_10_sec_intival[int(i+(t/10)-1)] = data_i_sec
    print(data_at_10_sec_intival)
    i+=10
    #make a file 10sec_data.csv and write the data_at_10_sec_intival to the file for every file in camera_data_uden_vicon

#save data_at_10_sec_intival as a csv file called 10sec_data.csv
np.savetxt("10sec_data.csv", data_at_10_sec_intival, delimiter=",")


# Plot the diff in x, y, and z position in mm as a function of time in seconds
plt.figure()
plt.plot(time, diff_x, label="Diff in x position")
plt.plot(time, diff_y, label="Diff in y position")
plt.plot(time, diff_z, label="Diff in z position")
plt.xlabel("Time (s)")
plt.ylabel("Diff in position (mm)")
plt.legend()
plt.grid()
plt.show()

# plot as a distance from the origin
distance = np.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
plt.figure()
plt.plot(time, distance, label="Distance from the origin")
plt.xlabel("Time (s)")
plt.ylabel("Distance (mm)")
plt.legend()
plt.grid()
plt.show()
# print the max distance
print("Max distance from the origin: ", np.max(distance))

# for time = 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 seconds get the x,y,z diff in mm and plot it
time_points = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100] #s
for t in time_points:
    idx = np.argmin(np.abs(time - t))
    print(f"Time: {t} s")
    print(f"Timestamp: {data[0].values[idx]}")
    print(f"Diff in x position: {diff_x[idx]} mm")
    print(f"Diff in y position: {diff_y[idx]} mm")
    print(f"Diff in z position: {diff_z[idx]} mm")
    print()

# save the 10 data points to a csv file


"""
# print the max abs diff in x, y, and z position
print("Max abs diff in x position: ", np.max(np.abs(diff_x)))
print("Max abs diff in y position: ", np.max(np.abs(diff_y)))
print("Max abs diff in z position: ", np.max(np.abs(diff_z)))
# print the mean abs diff in x, y, and z position
print("Mean abs diff in x position: ", np.mean(np.abs(diff_x)))
print("Mean abs diff in y position: ", np.mean(np.abs(diff_y)))
print("Mean abs diff in z position: ", np.mean(np.abs(diff_z)))
# print the std abs diff in x, y, and z position
print("Std abs diff in x position: ", np.std(np.abs(diff_x)))
print("Std abs diff in y position: ", np.std(np.abs(diff_y)))
print("Std abs diff in z position: ", np.std(np.abs(diff_z)))
"""

