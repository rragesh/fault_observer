cmake_minimum_required(VERSION 2.8.3)
project(fault_observer)
find_package(catkin REQUIRED COMPONENTS
	std_msgs
	ensenso_camera_msgs
	rospy
	message_generation
  message_runtime)

add_service_files(
  FILES
  fault_detect.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package()
