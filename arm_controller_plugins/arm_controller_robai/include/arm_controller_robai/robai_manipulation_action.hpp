/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/12/10
* 		- File created.
*
* Description:
*	For creating and executing Robai manipulation actions
* 
***********************************************************************************/
#ifndef ROBAI_MANIPULATION_ACTION_HPP
#define ROBAI_MANIPULATION_ACTION_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <vector>

#include <geometry_msgs/Pose.h>

#include "wait.hpp"

//robai
#include "manipulation/ecManipulationActionManager.h"
#include "remoteCommand/ecRemoteCommand.h"
#include "remoteCommandClientPlugin/remoteCommandPlugin.h"
#include "xml/ecXmlObjectReaderWriter.h"

namespace arm_controller_robai {  

//! @todo MdL: Make parameters for this.
#define MANIPULATOR_FILE          (ros::package::getPath("arm_controller_robai") + std::string("/config/manipulate_action.xml"))
#define MOVE_GRIPPER_TEMPLATE     (ros::package::getPath("arm_controller_robai") + std::string("/config/move_gripper_action_template.xml"))
#define MOVE_ARM_TEMPLATE         (ros::package::getPath("arm_controller_robai") + std::string("/config/move_arm_action_template.xml"))
#define MOVE_JOINTS_TEMPLATE      (ros::package::getPath("arm_controller_robai") + std::string("/config/move_joints_action_template.xml"))
#define MOVE_OBJECT_TEMPLATE      (ros::package::getPath("arm_controller_robai") + std::string("/config/move_object_action_template.xml"))
#define MAX_NR_TRIES 5
#define ENABLE_PATH_PLANNING_FUNCTION false

using boost::property_tree::ptree;
using std::vector;
using geometry_msgs::Pose;

using namespace Ec;

class RobaiManipulationAction
{
  public:
	RobaiManipulationAction();
	~RobaiManipulationAction();

    void setMaxManipulationTries(const int);
    void enablePathPlanningFunction(const bool);

    bool executePoseManipulation(const int& arm_index, const int& end_effector_index, const Pose& pose);
    bool executeGripperManipulation(const int& arm_index, const int& end_effector_index, const int& percentage_open);
    bool executeJointPositionManipulation();

	bool cancelManipulation();
	bool manipulationActive();

  private:

  	bool loadManipulationActionManager();
	bool readManipulationManagerFile();

	/**
     * Reads an XML file into a kdtree.
     * This is particularly used to read a manipulation action template and alter it (later on) for a particular action.
     * @param  filename The (full) XML file name.
     * @return          A KDTree storing all information of the XML file.
     */
    ptree readXml( const std::string& filename );

    /**
     * Writes a KDtree to an XML file.
     * This is particularly used to store a manipulation action back into the manipulator action manager file.
     * @param  filename The (full) filename to store the XML
     * @param  xml      The KDtree to store
     */
    bool writeXml( const std::string& filename, const ptree& xml );

    /**
     * Write the move to pose action to an XML file.
     * This is used particularly for path planning. It reads the MOVE_ARM_TEMPLATE file and stores pose into the MANIPULATION_FILE. Later on this
     * is executed by executeXmlAction().
     * @param  arm  For which arm to write a pose
     * @param  pose Which pose to write to the file.
     */
    bool writePoseToXml(const int& arm_index, const int& end_effector_index, const Pose& pose);

    /**
     * Write the gripper action to an XML file.
     * This is used particularly for setting the gripper width. It reads the MOVE_GRIPPER_TEMPLATE file and stores pose into the MANIPULATION_FILE. Later 
     * on this is executed by executeXmlAction().
     * @param  arm             For which arm to set the gripper width
     * @param  pertentage_open The percentage the gripper should be opened
     */
    bool writeGripperActionToXml( const int& gripper_index, const int& end_effector_index, const int& percentage_open);

    /**
     * Write the set joint positions action to an XML file.
     * This is used particularly for setting joint angles. It reads the MOVE_JOINTS_TEMPLATE file and stores it into the MANIPULATION_FILE. Later on this
     * is executed by executeXmlAction().
     * @param  arm             For which arm to write the joint positions
     * @param  joint_positions The particular joint positions
     * @param angle_tolerance  The tolerance of the angles reached
     */
    bool writeJointPositionsToXml( const int& arm_index, const vector<double>& joint_positions, const double& angle_tolerance );

    /**
     * Executes the action stored in the MANIPULATION_FILE.
     */
    bool executeXmlAction();

    bool executeAction();

    EcManipulatorEndEffectorPlacement convertToRobaiPose(const int& arm_index, const Pose& pose);

    bool waitForManipulationComplete();

    ros::NodeHandle n_;

    // Parameters
    int 	max_manipulation_tries_;
    bool 	enable_path_planning_function_;

    EcManipulationActionManager manager_;

    bool cancelled_;
    bool manipulation_active_;

};
};

#endif // ROBAI_MANIPULATION_ACTION_HPP 