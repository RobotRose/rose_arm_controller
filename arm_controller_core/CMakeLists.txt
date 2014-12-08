cmake_minimum_required(VERSION 2.8.3)
project(arm_controller_core)

find_package(catkin REQUIRED)

catkin_package(
	INCLUDE_DIRS 
		include
#  LIBRARIES arm_controller
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
