# P4_DroneProject
 
This guide is for the Jetson Nano - ubuntu 20.04 image. Written 03-2024.

The Flight controller being used is a PX4 Orange cube. 

For this step-by-step guide to work, you need to make sure your respective flight controller support PX4-ROS 2/DDS Bridge. One way to check this, is if the setting UXRCE_DDS_CFG can be found and set to TELEM2 - Using Qgroundcontrol.

Install image included in link on microSD (64 Gb is recommended - space for logging of flight data). Use a tool as BalenaEtcher. Link: https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image

Boot the Jetson Nano. Password: Jetson.

Start by fixing keyboard language if needed. And other Usersettings.

1. Fix the partition of the drive.
	Check if the drive only have part of its storage allocated.

Run in terminal:
	
	df -h

If expected valus are returned skip this part. Else:
 
 	sudo apt-get install gparted
	
Resize current partion until it is taking up as much of the available space as possible. Check again with
	
	df -h
	
2. Check CMake version. !!! DO THIS BEFORE INSTALLING ROS !!!
	Link: https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line
	
Run:
	
 	CMake --version
	
We want CMake 3.20 or later.
 	
Run:
 
 	sudo apt purge --autoremove cmake
 	sudo apt update && \
	sudo apt install -y software-properties-common lsb-release && \
	sudo apt clean all
 
	sudo apt upgrade
 	
 	wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
 	
 	sudo apt update
	sudo apt install kitware-archive-keyring
	sudo rm /etc/apt/trusted.gpg.d/kitware.gpg
	
Run "Sudo apt update" again to see if it worked. If kitware provides any key errors, you must fix it. (I had one, but fixed it by flashing new os).
	
	Sudo apt update
 
	sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 6AF7F09730B3F0A4
	
	sudo apt update
	sudo apt install cmake
	
	Finish by checking you new version with "cmake --version". I had cmake --version


3. PX4 User Guide.
	It is now time to follow the PX4 user-guide provided as: https://docs.px4.io/main/en/ros/ros2_comm.htmljhnjn 
	The goal in the end, is to talk with the pixhawk PX-4. !!!KEEP IN MIND THAT WE HAVE UBUNTU 20.04!!! We thus need to install ROS 2 FOXY.
	
 Install ROS2
	LINK: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

Run:
	
 	locale

If UTF-8, continue.
		
Run:
	
 	sudo apt install software-properties-common
	sudo add-apt-repository universe
		
If it returned expected results, then continue.
	
Run:
	
	sudo apt update && sudo apt install curl -y
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	sudo apt update
	sudo apt upgrade
	
If it returned expected results, then continue.
	

Install ROS2 FOXY with desktop:
	
 	sudo apt install ros-foxy-desktop python3-argcomplete
	sudo apt install ros-dev-tools
		
Install colcon (should be installed with ros-dev-tools):
	
	sudo apt install python3-colcon-common-extensions
	
If it returned expected results, then continue. 
	
Install Python dependencies:
	
	pip install --user -U empy==3.3.4 pyros-genmsg setuptools
	

				
Setup Micro XRCE-DDS Agent:
For ROS 2 to communicate with PX4, uXRCE-DDS client must be running on PX4, connected to a micro XRCE-DDS agent running on the companion computer. The Simulater tool has its own client to simulate, and will itself start it. As we dont need the simulator tool, there is no need to run the client. We now focus on setting up the agent.
		

Setup the Agent. Open a new terminal:
	
	git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
	cd Micro-XRCE-DDS-Agent
	mkdir build
	cd build
	cmake ..
	make
	sudo make install
	sudo ldconfig /usr/local/lib/

Building workspace - Chose own name. Replace P4DroneProject:
	
	mkdir -p ~/P4DroneProject/src/
	cd ~/P4DroneProject/src/
	
	git clone https://github.com/PX4/px4_msgs.git
	git clone https://github.com/PX4/px4_ros_com.git
	
	cd ..
	source /opt/ros/foxy/setup.bash
	colcon build
 
 
For the agent to talk with the Flight Controller, a connection needs to be established. We will do it trough serial. On the Flight controller, a micro-usb connector is found. This is only used to set it up. Instead the Telem2 port on the flight-controller should be used. 

!!WARNING!!

In the setup using QGroundControl make sure to change the settings under Parameters:

	UXRCE_DDS_CFG : TELEM2
	SER_TEL2_BUAD : 115200 8N1

If you are not able to find this setting, the flight controller is most likely not supporting the PX4-ROS 2/DDS Bridge

Now the TELEM 2 port should be connected to the jetson nano with the RX, TX and GND pins from the TELEM 2 port. Follow the guides:
https://www.hackster.io/Matchstic/connecting-pixhawk-to-raspberry-pi-and-nvidia-jetson-b263a7
https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html#serial-port-setup


Try example:

	Find USB:
	
	dmesg | grep tty
		
		like: ttyACM0
	
	Run:
	sudo MicroXRCEAgent serial --dev /dev/ttyACM0 -b 115200
	
	cd ~/P4DroneProject/
	source /opt/ros/foxy/setup.bash
	source install/local_setup.bash
	
	
This should yield some Gyro and accelerometer data.


Now it is time to develop your own application.
