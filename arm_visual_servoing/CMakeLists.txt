cmake_minimum_required(VERSION 2.8.3)
project(arm_visual_servoing)

find_package(catkin REQUIRED
	COMPONENTS
	rose_arm_controller_msgs
	server_multiple_client
	tf
)

catkin_package(
 	INCLUDE_DIRS include
#  LIBRARIES arm_visual_servoing
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(arm_visual_servoing 	${PROJECT_SOURCE_DIR}/src/arm_visual_servoing_node.cpp
									${PROJECT_SOURCE_DIR}/src/arm_visual_servoing.cpp
)
target_link_libraries(arm_visual_servoing ${catkin_LIBRARIES})
add_dependencies(arm_visual_servoing ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)