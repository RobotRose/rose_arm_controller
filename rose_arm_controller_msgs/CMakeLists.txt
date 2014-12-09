cmake_minimum_required(VERSION 2.8.3)
project(rose_arm_controller_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin 
	REQUIRED
		message_generation 
		actionlib_msgs 
		geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  #Message1.msg
)

# Generate services in the 'srv' folder
add_service_files(
	FILES
		emergency_stop.srv
		reset_emergency_stop.srv
)

# Generate actions in the 'action' folder
add_action_files(
	FILES
		set_position.action
		set_velocity.action
		set_gripper_width.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
 	DEPENDENCIES
		actionlib_msgs
		geometry_msgs
		std_msgs 
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rose_arm_controller_msgs
	CATKIN_DEPENDS 
		actionlib_msgs 
		geometry_msgs
#  DEPENDS system_lib
)