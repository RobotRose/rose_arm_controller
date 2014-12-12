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
#include "arm_controller_robai/robai_manipulation_action.hpp"

namespace arm_controller_robai {    

RobaiManipulationAction::RobaiManipulationAction()
    : n_("~robai_manipulation_action")
{
    // Create private node handle

    loadParameters();
}

RobaiManipulationAction::~RobaiManipulationAction()
{

}

bool RobaiManipulationAction::executePoseManipulation(const int& arm_index, const int& end_effector_index, const Pose& pose)
{
    cancelled_ = false;

    ROS_DEBUG("RobaiManipulationAction::moveToPose arm %d", arm_index);
    ROS_DEBUG("RobaiManipulationAction::moveToPose pose (%f, %f, %f) ; (%f, %f, %f, %f)",
                    pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                    pose.orientation.y, pose.orientation.z, pose.orientation.w);

    // setEndEffectorMode(arm, mode);
    // EcSLEEPMS(100);

    // geometry_msgs::Pose corrected_pose = correctedEndEffectorPose(pose);
    Pose corrected_pose = pose; //! @todo MdL: Find out what to do here.

    bool result = false;
    if (enable_path_planning_function_parameter_ && not cancelled_)
    {
        if (not setPathPlanningDesiredPlacement(convertToRobaiPose(arm_index, corrected_pose)))
            ROS_ERROR("Could not set path planning pose.");
        // wait
        result = waitForManipulationComplete();
    }

    // If setPathPlanningDesiredPlacement fails
    if (not result && not cancelled_)
    {
        ROS_DEBUG("Doing (XML) manipulation action, setPathPlanningDesiredPlacement() has failed.");
        writePoseToXml(arm_index, end_effector_index, corrected_pose);
        result = executeAction();
    }

    return result;
}

bool RobaiManipulationAction::executeGripperManipulation(const int& arm_index, const int& end_effector_index, const int& percentage_open)
{
	cancelled_ = false;

    ROS_DEBUG("RobaiManipulationAction::moveGripper");
    writeGripperActionToXml(arm_index, end_effector_index, percentage_open);

    return executeAction();
}

bool RobaiManipulationAction::executeJointPositionManipulation()
{
	ROS_ERROR("Not yet implemented");
	return false;
}

bool RobaiManipulationAction::cancelManipulation()
{
    ROS_DEBUG("Setting cancel variable");
	cancelled_ = true;
    
    if (manipulation_active_)
        if (not stopManipulation()) 
            ROS_ERROR("Could not stop manipulation");

	return true;
}

bool RobaiManipulationAction::manipulationActive()
{
	return manipulation_active_;
}

bool RobaiManipulationAction::loadParameters()
{
    ROS_INFO("Loading robai arm parameters...");

    //! @todo MdL: Check is values are loaded from configuration.
    // if(not )
    //     ROS_WARN("Maximal number of manipulation tries was not set in confugation file, defaulting to %d", max_manipulation_tries_parameter_);

    // if(not )
    //     ROS_WARN("Enable/Disable path planning function not set, not using function");

    n_.param("/robai_configuration/max_manipulation_tries", max_manipulation_tries_parameter_, 1);
    n_.param("/robai_configuration/enable_path_planning_function", enable_path_planning_function_parameter_, false);

    ROS_INFO("Done.");

    return true;
}

bool RobaiManipulationAction::loadManipulationActionManager()
{
    EcManipulationActionManager::StringActionMap map;
    manager_.setActionMap(map);

    ROS_DEBUG("Setting manipulation file");
    if (not setManipulationPathFile(MANIPULATOR_FILE))
        ROS_ERROR("Could not set manipulation file.");

    ROS_DEBUG("Reading manipulation file");
    readManipulationManagerFile();

    if (not setManipulationActionManager(manager_))
        ROS_ERROR("Could not set manipulation action manager");

    ROS_DEBUG("Found %d actions", manager_.actionMap().size());
}

bool RobaiManipulationAction::readManipulationManagerFile()
{
    ROS_DEBUG("RobaiManipulationAction::readManipulationManagerFile()");
    std::string manipulation_action_file = MANIPULATOR_FILE;

    if (not EcXmlObjectReaderWriter::readFromFile(manager_, manipulation_action_file))
        ROS_ERROR("Unable to load action manager from file");
}

ptree RobaiManipulationAction::readXml(const std::string& filename)
{
    using boost::property_tree::ptree;
    ptree pt;
    read_xml(filename, pt, boost::property_tree::xml_parser::trim_whitespace);

    return pt;
}

bool RobaiManipulationAction::writeXml(const std::string& filename, const boost::property_tree::ptree& xml)
{
    boost::property_tree::xml_writer_settings<char> settings(' ', 4);
    write_xml(filename, xml, std::locale(), settings);

    return true;
}

bool RobaiManipulationAction::writePoseToXml(const int& arm_index, const int& end_effector_index, const Pose& pose)
{
    ROS_DEBUG("RobaiManipulationAction::writePoseToXml( %s )",
                    MOVE_ARM_TEMPLATE.c_str());
    ROS_DEBUG("pose.position.x: %f", pose.position.x);
    ROS_DEBUG("pose.position.y: %f", pose.position.y);
    ROS_DEBUG("pose.position.z: %f", pose.position.z);

    ptree pt = readXml(MOVE_ARM_TEMPLATE);

    ptree& path_planning_action = pt.get_child(
        "manipulationActionManager.actionMap.element.value.endEffectorPathPlanningAction");

    path_planning_action.put("activeEndEffectorSet", end_effector_index);
    path_planning_action.put("endEffectorIndex", arm_index);

    ptree& orientation = path_planning_action.get_child("goal.cr:orientation");
    ptree& orientation_attributes = orientation.get_child("<xmlattr>");

    orientation_attributes.put<double>("q0", pose.orientation.w);
    orientation_attributes.put<double>("q1", pose.orientation.x);
    orientation_attributes.put<double>("q2", pose.orientation.y);
    orientation_attributes.put<double>("q3", pose.orientation.z);

    ptree& xml_pose = path_planning_action.get_child("goal.cr:translation");
    ptree& position_attributes = xml_pose.get_child("<xmlattr>");
    position_attributes.put<double>("x", pose.position.x);
    position_attributes.put<double>("y", pose.position.y);
    position_attributes.put<double>("z", pose.position.z);

    return writeXml(MANIPULATOR_FILE, pt);
}

bool RobaiManipulationAction::writeGripperActionToXml(const int& gripper_index, 
													  const int& end_effector_index, 
												 	  const int& percentage_open)
{
    ROS_DEBUG("RobaiManipulationAction::writeGripperActionToXml( %s )",
                    MOVE_GRIPPER_TEMPLATE.c_str());

    ptree pt = readXml(MOVE_GRIPPER_TEMPLATE);

    ptree& gripper_action =
        pt.get_child("manipulationActionManager.actionMap.element.value.gripperAction");

    gripper_action.put<int>("activeEndEffectorSet", end_effector_index);
    gripper_action.put<int>("endEffectorIndex", gripper_index);
    gripper_action.put<int>("gripperDesiredPositionPercentage", percentage_open);

    return writeXml(MANIPULATOR_FILE, pt);
}

bool RobaiManipulationAction::writeJointPositionsToXml(const int& arm_index,
                                                  const std::vector<double>& joint_positions,
                                                  const double& angle_tolerance)
{
    // ROS_DEBUG("RobaiManipulationAction::writeJointPositionsToXml( %s )",
    //                 MOVE_JOINTS_TEMPLATE.c_str());

    // ptree pt = readXml(MOVE_JOINTS_TEMPLATE);

    // ptree& joint_path_planning_action =
    //     pt.get_child("manipulationActionManager.actionMap.element.value.jointPathPlanningAction");
    // ptree& goal_vector =
    //     joint_path_planning_action.get_child("labelledGoalVector.element.second.mn:jointPositions");

    // std::string joint_angles = "";
    // for (int i = 0; i < joint_positions.size(); i++)
    // {
    //     joint_angles += " " + rose20_common::doubleToString(joint_positions[i]);
    //     if (i == 7 || i == 15)
    //         joint_angles += " " + rose20_common::doubleToString(joint_positions[i]);
    // }

    // goal_vector.put<std::string>("mn:group", joint_angles);

    // writeXml(MANIPULATOR_FILE, pt);
    return false;
}

bool RobaiManipulationAction::executeXmlAction()  //! @todo MdL: Remove parameter
{
    EcXmlStringVector action_list = manager_.actionOrder();

    // There is only one action in the manipulation file
    ROS_DEBUG("Doing action: %s", action_list[0].value().c_str());

    if (not setManipulationAction(action_list[0].value()))
    {
        ROS_ERROR("Could not set manipulation action");
        return false;
    }

    if (not startManipulation())
    {	
    	ROS_ERROR("Could not start manipulation");
    	return false;
    }

    ROS_DEBUG("Manipulation sent");
    return true;
}

bool RobaiManipulationAction::executeAction()
{
    manipulation_active_ = true;

    ROS_DEBUG("RobaiManipulationAction::executeAction");
    loadManipulationActionManager();

    bool result = false;
    int nr_fails = 0;
    while (not result && nr_fails < max_manipulation_tries_parameter_ && not cancelled_)
    {
        executeXmlAction();

        result = waitForManipulationComplete();

        // result = true will exit the loop
        nr_fails++;
    }

    ROS_DEBUG("RobaiManipulationAction::executeAction result");
    manipulation_active_ = false;

    return result;
}

EcManipulatorEndEffectorPlacement RobaiManipulationAction::convertToRobaiPose(const int& arm_index, const Pose& pose)
{
    EcManipulatorEndEffectorPlacement desired_placement;
    //! @todo MdL: Can I disregard the following (commented lines)?
    // EcSLEEPMS(300);
    // desired_placement = actual_arms_placement_;

    EcCoordinateSystemTransformation system_transformation;
    system_transformation.setTranslation(
        EcVector(pose.position.x, pose.position.y, pose.position.z));

    system_transformation.setOrientation(EcOrientation(pose.orientation.w, pose.orientation.x,
                                                       pose.orientation.y, pose.orientation.z));

    desired_placement.offsetTransformations()[arm_index].setCoordSysXForm(system_transformation);

    return desired_placement;
}

bool RobaiManipulationAction::waitForManipulationComplete()
{
    Wait wait;
    bool manipulation_successful;
    setManipulationCompletedCallback(boost::bind(&Wait::CB_actionCompleted, &wait, _1, _2));

    if (wait.waitForCompletion())
    {
        manipulation_successful = true;
        ROS_DEBUG("Manipulation successful");
    }
    else
    {
        manipulation_successful = false; 
        ROS_DEBUG("Manipulation FAILED!");
    }

    // if ( not resetEndEffectorSet()) 
    // 	ROS_ERROR("Could not reset end effector mode");
    EcSLEEPMS(200);

    return manipulation_successful;
}

};