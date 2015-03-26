cmake_minimum_required(VERSION 2.8.3)
project(arm_controller_kinova_wpi)

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

# Kinova base
add_library( arm_controller_kinova
	src/arm_controller_kinova.cpp
)

add_dependencies(arm_controller_kinova ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Mico arm
add_library( arm_controller_mico
	src/arm_controller_mico.cpp
)

add_dependencies(arm_controller_mico ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(arm_controller_mico arm_controller_kinova)

# Jaco arm
add_library( arm_controller_jaco
	src/arm_controller_jaco.cpp
)

add_dependencies(arm_controller_jaco ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(arm_controller_jaco arm_controller_kinova)


install( 
	TARGETS 
		arm_controller_jaco 
		arm_controller_mico 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)

install(
	FILES 
		mico_plugin.xml 
		jaco_plugin.xml
		kinova_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)