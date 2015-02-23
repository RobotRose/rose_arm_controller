cmake_minimum_required(VERSION 2.8.3)
project(arm_controller_robai)

find_package(catkin 
	REQUIRED
	COMPONENTS
		arm_controller_base
		roscpp
		geometry_msgs
		pluginlib
		roscpp
		rose_arm_controller_msgs
		rose_third_party
		tf
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
		rose_rose_third_party
		tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

link_directories(
	${rose_third_party_INCLUDE_DIRS}/../lib/ 
	# ${Boost_LIBS}
)

set(REMOTE_COMMAND_CLIENT ${rose_third_party_INCLUDE_DIRS}/../bin/remoteCommandClientPlugin.ecp)
set(MANIPULATION_PLUGIN   ${rose_third_party_INCLUDE_DIRS}/../bin/manipulationActionExecPlugin.ecp)
set(CYTON_PLUGIN          ${rose_third_party_INCLUDE_DIRS}/../bin/cytonPlugin.ecp)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${rose_third_party_INCLUDE_DIRS}
)

add_library( arm_controller_robai
	src/arm_controller_robai.cpp
	src/robai_manipulation_action.cpp
)

target_link_libraries( arm_controller_robai
	# ecActinSENetwork
	# ecActinSE
	# ecCommonControls
	# ecControl
	# ecConvertSimulation
	# ecConvert
	# ecConvertSystem
	# ecCytonBase
	# ecCytonController
	# ecCytonHardwareInterface
	# ecCytonViewerBase
	# ecDragger
	# ecFilterStream
	# ecForceControl
	# ecFoundCommon
	# ecFoundCore
	# ecFunction
	# ecGeometry
	# ecGrasping
	# ecHardwarePlugin
	# ecImageCapture
	# ecImageFilter
	# ecImagePipeline
	# ecImageSegmentationUI
	# ecImageSensorDisplay
	# ecImageSensor
	# ecImage
	# ecImageTransform
	# ecInterface
	# ecLoader
	# ecManipulationActionManagerGui
	# ecManipulationDirectorGui
	# ecManipulationDirector
	# ecManipulationSequenceEditor
	# ecManipulation
	# ecManipulator
	# ecMatrixUtilities
	# ecMeasure
	# ecPathPlanning
	# ecPlanarGeometry
	# ecPluginCommon
	# ecPlugins
	ecRemoteCommand
	# ecRendCore
	# ecRender
	# ecSensCore
	# ecSensMeas
	# ecSensorInterface
	# ecSerial
	# ecShapeProximity
	# ecSimulationAnalysis
	# ecSimulation
	# ecSlotManager
	# ecSocket
	# ecSoilMechanics
	# ecStream
	# ecTransport
	# ecUserManagement
	# ecViewerCore
	# ecVisualization
	# ecVrml97
	# ecWalking
	# ecXmlReaderWriter
	# ecXml
	${catkin_LIBRARIES}
	${CYTON_PLUGIN}
	${MANIPULATION_PLUGIN}
	${PROJECT_NAME}
	${REMOTE_COMMAND_CLIENT}
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