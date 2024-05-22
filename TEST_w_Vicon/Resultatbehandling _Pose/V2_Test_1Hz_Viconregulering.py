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

start=True
i=0
for file in camera_data_uden_vicon:
    if start:
        #make len(camera_data_uden_vicon)x4 array
        data_at_10_sec_intival = np.zeros((len(camera_data_uden_vicon)*10,6))
        start=False
    path_to_file = os.path.join(path, file)
    data = pd.read_csv(path_to_file, header=None)

    time = data[0].values

    diff_x = data[1].values
    diff_y = data[2].values
    diff_z = data[3].values
    roll_cam = data[4].values
    pitch_cam = data[5].values
    yaw_cam = data[6].values
    roll_vicon = data[7].values
    pitch_vicon = data[8].values
    yaw_vicon = data[9].values

    # Convert the timestamp to seconds
    time = (time - time[0]) / 1000

    t_old=0
    diff_x_regulering =diff_x[0]
    diff_y_regulering =diff_y[0]
    diff_z_regulering =diff_z[0]

    diff_x_list = []
    diff_y_list = []
    diff_z_list = []
    time_list = []
    diff_yaw_list = []
    for t in time:
        #get t index
        idx = np.argmin(np.abs(time - t))
        if t-t_old >= 1.0:
            t_old=t
            diff_x_regulering=diff_x[idx]
            diff_y_regulering=diff_y[idx]
            diff_z_regulering=diff_z[idx]
        #append diff_x[idx]-diff_x_regulering to diff_x_list
        time_list.append(t)
        diff_x_list.append(diff_x[idx]-diff_x_regulering)
        diff_y_list.append(diff_y[idx]-diff_y_regulering)
        diff_z_list.append(diff_z[idx]-diff_z_regulering)
        diff_yaw_list.append(np.rad2deg(yaw_cam[idx]-yaw_vicon[idx]))

    time_points = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100] #s
        # for time = 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 seconds get the x,y,z diff in mm and wirte it to a csv file called 10sec_data.csv where the first column is time in seconds and the second, third, and fourth column is the diff in x, y, and z position in mm

    for t in time_points:
        idx = np.argmin(np.abs(time - t))
        #print("i",i)
        #print("t",t)
        #print("time[idx]",time[idx])
        #print("index",idx)
        data_i_sec = np.array([time[idx], diff_x_list[idx], diff_y_list[idx], diff_z_list[idx],np.sqrt(diff_x_list[idx]**2 + diff_y_list[idx]**2 + diff_z_list[idx]**2),diff_yaw_list[idx]])
        data_at_10_sec_intival[int(i+(t/10)-1)] = data_i_sec
    i+=10
print(data_at_10_sec_intival)
np.savetxt("10sec_data_Vicon_reguleret.csv", data_at_10_sec_intival, delimiter=",")

plt.figure()
plt.bar(data_at_10_sec_intival[:,0], data_at_10_sec_intival[:,-2], label="Distance from the origin")
plt.xlabel("Time (s)")
plt.ylabel("diff Distance (mm)")
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.bar(data_at_10_sec_intival[:,0], data_at_10_sec_intival[:,-1], label="Distance from the origin")
plt.xlabel("Time (s)")
plt.ylabel("diff Yaw (deg)")
plt.legend()
plt.grid()
plt.show()

