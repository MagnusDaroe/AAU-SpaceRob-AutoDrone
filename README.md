# AAU-SpaceRob-AutoDrone
This guide is for the Jetson Nano - ubuntu 20.04 image. Written 03-2024.

The goal of this project is to control a drone with an onboard computer. In this case the Jetson Nano.
 
The flight controller being used is a Pixhawk radiolink

Download image included in link on microSD (64 Gb is recommended - space for logging of flight data). Use a tool as BalenaEtcher to flash image onto microSD. Link: https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image

Boot the Jetson Nano. Password: Jetson.

Start by fixing keyboard language if needed. And other user settings.

## 1. Fix the partition of the drive.
	Check if the drive only have part of its storage allocated.

Run in terminal:
	
	df -h

If expected valus are returned skip this part. Else:
 
 	sudo apt-get install gparted
	
Resize current partion until it is taking up as much of the available space as possible. Check again with
	
	df -h
	
## 2. Check CMake version. !!! DO THIS BEFORE INSTALLING ROS !!!
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

## 3. Install ROS2
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
