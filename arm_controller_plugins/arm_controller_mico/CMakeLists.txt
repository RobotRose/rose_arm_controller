cmake_minimum_required(VERSION 2.8.3)
project(arm_controller_mico)

find_package(catkin 
	REQUIRED
	COMPONENTS
		arm_controller_base
		roscpp
		geometry_msgs
		pluginlib
		roscpp
		rose_arm_controller_msgs
		tf
		wpi_jaco_msgs
)

catkin_package(
  	INCLUDE_DIRS 
  		include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS 
		arm_controller_base
		geometry_msgs
		pluginlib
		roscpp
		rose_arm_controller_msgs
		tf
		wpi_jaco_msgs
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME}
	src/arm_controller_mico.cpp
)

install(TARGETS ${PROJECT_NAME} 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)

install(FILES mico_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)