cmake_minimum_required(VERSION 2.8.3)
project(arm_controller_base)

find_package(catkin 
	REQUIRED
		COMPONENTS
		geometry_msgs
		std_msgs
		tf
)

catkin_package(
	INCLUDE_DIRS 
		include
#  LIBRARIES arm_controller_base
	CATKIN_DEPENDS 
        geometry_msgs
		std_msgs
		tf
#  DEPENDS system_lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
