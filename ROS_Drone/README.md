# Workspace til Quadcopter med ROS2

følger denne:
https://docs.px4.io/main/en/ros/ros2_comm.html


for at bygge:
cd sti/til/ROS_Drone
source /opt/ros/humble/setup.bash
colcon build


for at køre programmet:
cd ~/PX4-Autopilot && make px4_sitl gz_x500  <---------- hvis det skal være med gazebo. ellers undlad.

cd sti/til/ROS_Drone
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch px4_ros_com >>>dit_yndlings_launchscript<<<.launch.py




