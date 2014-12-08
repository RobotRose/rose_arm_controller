cmake_minimum_required(VERSION 2.8.3)
project(arm_controller)

find_package(catkin REQUIRED
	COMPONENTS
		pluginlib
        tf
)

catkin_package(
	INCLUDE_DIRS 
		include
#  LIBRARIES arm_controller
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(arm_controller
	src/arm_controller.cpp
	src/arm_controller_node.cpp
)

target_link_libraries(arm_controller 
	${catkin_LIBRARIES}
)

include_directories(
	${catkin_INCLUDE_DIRS} 
)

install(
	DIRECTORY 
		launch
	DESTINATION 
		${CATKIN_PACKAGE_SHARE_DESTINATION}
	USE_SOURCE_PERMISSIONS
)

install(
    TARGETS
        arm_controller
    LIBRARY DESTINATION 
    	${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION 
    	${CATKIN_PACKAGE_BIN_DESTINATION}
)