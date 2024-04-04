# AAU-SpaceRob-AutoDrone
This repository contains the code for Robotics Group 465 at AAU. The project was developed in collaboration with AAU Space Robotics and serves as a foundation for further development in preparation for the upcoming ERC competition.

### HARDWARE:
- Unknown Carbon Fiber Drone Kit (V-shape)
- Motors: Air 2213 kv920 with props.
- Speed Controllers: Unknown
- DC-to-DC Converter: UBEC 5A HV
- Battery: Turnigy 4-cell battery, 4000 mAh - LIPO
- Onboard Computer: Jetson Nano

### SOFTWARE:
- Operating System: Jetson Nano Ubuntu 20.04 image
- ROS2 Distribution: FOXY - Developed our own package.

To get started, download the image from the link provided onto a microSD card (64 GB recommended for flight data logging). Use a tool like BalenaEtcher to flash the image onto the microSD card. Here’s the link: https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image


# Guide
Written 04-2024.

This guide is for the Jetson Nano - Ubuntu 20.04 image. The goal of this project is to control a drone autonomously with an onboard computer, such that it can compete in the ERC competetion.
 
Flash the image and boot the Jetson Nano. Password: Jetson.

Start by fixing keyboard language if needed. And other user settings.

## 1. Fix the partition of the drive.
Check if the drive only have part of its storage allocated. In our case only 26 GB was partioned.

Run in terminal:
	
	df -h

If expected valus are returned skip this part. Else run the following command:
 
 	sudo apt-get install gparted
	
Resize current partion until it is taking up as much of the available space as possible. Check again with:
	
	df -h
	
## 2. Check CMake version. 
It is imperative the CMake version is updated to the newest available version for our setup. Else there will be problems installing ROS2.

Link: 
	
 	https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line
	
Run:
	
 	CMake --version
	
We want CMake 3.20 or later. If it is newer than this version, you can skip section 2.
 	
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

## 3. Install ROS2
If the ROS2 installation guide in the following link does not work. Use ours based on second link.

Installation guide:

	https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

Key issue fix:

 	https://answers.ros.org/question/398460/how-to-add-a-pubkey/

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
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
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

 You should now have working ROS2 Distrubution on the onboard computer. 
 
## 4. Build a ROS2 workspace on the Onboard computer.



 


 
