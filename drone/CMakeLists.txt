	cmake_minimum_required(VERSION 3.5)
	project(drone)

	# Default to C++14
	if(NOT CMAKE_CXX_STANDARD)
	  set(CMAKE_CXX_STANDARD 14)
	endif()

	if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	  add_compile_options(-Wall -Wextra -Wpedantic)
	endif()

	# Find dependencies
	find_package(ament_cmake REQUIRED)
	find_package(ament_cmake_python REQUIRED)
	find_package(rclcpp REQUIRED)
	find_package(rclpy REQUIRED)
	
	find_package(geometry_msgs REQUIRED)
	find_package(std_msgs REQUIRED)
	find_package(sensor_msgs REQUIRED)
	
	find_package(builtin_interfaces REQUIRED)
	find_package(rosidl_default_generators REQUIRED)
	if(NOT builtin_interfaces_FOUND)
    		message(FATAL_ERROR "Could not find builtin_interfaces package")
	endif()

	#find_package(drone REQUIRED)  

	rosidl_generate_interfaces(${PROJECT_NAME}
	  "msg/DroneCommand.msg"
	  "msg/DroneControlData.msg"
	  "msg/DroneStatus.msg"
	  "msg/DroneVideo.msg"
	  "srv/Clock.srv"
	  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs)
	# Include Cpp "include" directory
	include_directories(include)

	# Create Cpp executable
	add_executable(cpp_executable src/drone_control_node.cpp) # change name of node
	ament_target_dependencies(cpp_executable rclcpp)
	# Install Cpp executables
	install(TARGETS
	  cpp_executable
	  DESTINATION lib/${PROJECT_NAME}
	)
	# Install Python modules
	ament_python_install_package(${PROJECT_NAME})
	# Install Python executables
	install(PROGRAMS
	  scripts/fc_cmd.py
	  scripts/clock.py
	  scripts/manual_cmd.py
	  scripts/image_publish.py
	  scripts/image_listen.py
	  scripts/GCS.py
	  scripts/viconData.py
	  scripts/time.py
	  DESTINATION lib/${PROJECT_NAME} # change name of node in python
	)

	# Install launch files.
	install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
	)
	ament_package()
