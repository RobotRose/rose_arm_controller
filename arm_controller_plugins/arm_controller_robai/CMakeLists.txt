cmake_minimum_required(VERSION 2.8.3)
project(arm_controller_robai)

find_package(catkin 
	REQUIRED
	COMPONENTS
		arm_controller_base
		rose_arm_controller_msgs
		geometry_msgs
		pluginlib
		roscpp
)

catkin_package(
  	INCLUDE_DIRS 
  		include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS 
		arm_controller_base
		rose_arm_controller_msgs
		geometry_msgs
		pluginlib
		roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable( arm_controller_robai
	src/arm_controller_robai.cpp
)
target_link_libraries( arm_controller_robai
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} arm_controller_robai
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)

install(FILES robai_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)