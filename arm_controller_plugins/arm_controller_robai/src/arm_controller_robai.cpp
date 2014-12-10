/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Mathijs de Langen
*   Date  : 2014/12/08
*       - File created.
*
* Description:
*   This package describes the robai interface for robot arms.
* 
***********************************************************************************/
#include "arm_controller_robai/arm_controller_robai.hpp"

namespace arm_controller_plugins {

ArmControllerRobai::ArmControllerRobai()
	: end_effector_mode_(POINT_EE)
	, control_mode_(POSITION)
	, manipulation_action_manager_()
{

}

ArmControllerRobai::~ArmControllerRobai()
{

}

bool ArmControllerRobai::initialize()
{
	ROS_INFO("Initializing Robai arm...");
	return connectToArms();
}

bool ArmControllerRobai::close()
{
	shutdown();
	return true;
}

bool ArmControllerRobai::cancel()
{
	//! @todo MdL: Implement.
	return false;
}

bool ArmControllerRobai::emergencyStop()
{
	//! @todo MdL: Implement.
	return false;
}

bool ArmControllerRobai::resetEmergencyStop()
{
	//! @todo MdL: Implement.
	return false;
}

Pose ArmControllerRobai::getEndEffectorPose()
{
	//! @todo MdL: Implement.
	Pose pose;
	return pose;
}

bool ArmControllerRobai::setEndEffectorPose(const Pose& end_effector_pose)
{	
	ROS_INFO("ArmControllerRobai::setEndEffectorPose: (%f,%f,%f)",
                    end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z);
	return manipulation_action_manager_.executePoseManipulation(getRobaiArmIndex(), getRobaiEndEffectorMode(), end_effector_pose);
}

Twist ArmControllerRobai::getEndEffectorVelocity()
{
	//! @todo MdL: Implement.
	Twist twist;
	return twist;
}

bool ArmControllerRobai::setEndEffectorVelocity(const Twist& velocity)
{
	ROS_INFO("ArmControllerRobai::setVelocity: (%f,%f,%f):(%f,%f,%f)",
                    velocity.linear.x, velocity.linear.y, velocity.linear.z, velocity.angular.x,
                    velocity.angular.y, velocity.angular.z);

    if ( not setVelocityControl())
    	return false;

    if (getEndEffectorMode() != FRAME_EE) 
    	if ( not setEndEffectorMode(FRAME_EE))
    		return false;

    std::vector<double> velocity_vector;
    velocity_vector.push_back(velocity.linear.x);
    velocity_vector.push_back(velocity.linear.y);
    velocity_vector.push_back(velocity.linear.z);

    velocity_vector.push_back(velocity.angular.x);
    velocity_vector.push_back(velocity.angular.y);
    velocity_vector.push_back(velocity.angular.z);

    if (not setDesiredVelocity(velocity_vector, ARM, getRobaiArmIndex()))
    {
        ROS_ERROR("Could not set desired velocity");
        return false;
    }

	return true;
}

Twist ArmControllerRobai::getConstraints()
{
	Twist twist;
	if (getEndEffectorMode() == FRAME_EE)
	{
		//! @todo MdL: Verify.
		twist.angular.x = 1.0;
		twist.angular.y = 1.0;
		twist.angular.z = 1.0;
	}
	else if (getEndEffectorMode() == FREE_SPIN)
	{
		//! @todo MdL: Verify.
		twist.angular.x = 1.0;
		twist.angular.y = 1.0;
	}
	else if (getEndEffectorMode() == POINT_EE)
	{
		// Do nothing (all zero values)
	}
	else
		ROS_ERROR("Could not get constraints");
	
	return twist;
}

bool ArmControllerRobai::setConstraints(const Twist& constraint)
{
	ROS_INFO("setConstraints request received");

	if (constraint.linear.x > 0)
		ROS_WARN("The robai arm does not allow for setting linear constraints");
	if (constraint.linear.y > 0)
		ROS_WARN("The robai arm does not allow for setting linear constraints");
	if (constraint.linear.z > 0)
		ROS_WARN("The robai arm does not allow for setting linear constraints");
	if (constraint.angular.x > 0)
		if (constraint.angular.y > 0)
			if (constraint.angular.z > 0)
				return setEndEffectorMode(FRAME_EE);
			else
				return setEndEffectorMode(FREE_SPIN); //! @todo MdL: Verify.
		else // angular.y == 0
			ROS_WARN("No mode possible for angular constraint: (%f, %f, %f)", constraint.angular.x, constraint.angular.y, constraint.angular.z);
	else
		if (constraint.angular.y > 0)
			ROS_WARN("No mode possible for angular constraint: (%f, %f, %f)", constraint.angular.x, constraint.angular.y, constraint.angular.z);	
		if (constraint.angular.z > 0)
			ROS_WARN("No mode possible for angular constraint: (%f, %f, %f)", constraint.angular.x, constraint.angular.y, constraint.angular.z);
		return setEndEffectorMode(POINT_EE);

	return false;
}

bool ArmControllerRobai::resetConstraints()
{
	return setEndEffectorMode(POINT_EE);
}

double ArmControllerRobai::getGripperWidth()
{
	//! @todo MdL: Implement.
	return 0.0;
}

bool ArmControllerRobai::setGripperWidth(const double required_width)
{	
	//! @todo MdL: Map width to percentage min/max width min_gripper width and max gripper width.
	// For now: Smaller than 0.01 is closed, more than 0.0 is open
	int percentage_open = (required_width < 0.01 ? 0: 100);
	return manipulation_action_manager_.executeGripperManipulation(getRobaiGripperIndex(), getRobaiEndEffectorMode(), percentage_open);
}

Wrench ArmControllerRobai::getEndEffectorWrench()
{
	ROS_ERROR("Robai does not support reading force/torque values");
	Wrench wrench;
	return wrench;
}

bool ArmControllerRobai::setEndEffectorWrench(const Wrench& Wrench)
{
	//! @todo MdL: Implement.
	ROS_ERROR("Robai does not support force/torque control");
	return false;
}

JointState ArmControllerRobai::getJointStates()
{
	//! @todo MdL: Implement.
	JointState joint_states;
	return joint_states;
}

bool ArmControllerRobai::connectToArms(const std::string ip)
{
    ROS_INFO("Connecting to arms...");

    ros::Rate r(1);

    bool connected = false;
    while (not connected)
    {
        try
        {
            if (init(ip))
            {
                connected = true;
                ROS_DEBUG("Connected to ip %s", ip.c_str());
            }
        }
        catch (const boost::thread_interrupted& ex)
        {
            ROS_ERROR("Received a boost::thread_interrupted exception");

            ROS_DEBUG("Could not connect to ip %s", ip.c_str());
            ROS_DEBUG("Retrying");
        }

        r.sleep();
    }
    ROS_INFO("Connected!");
    return true;
}

bool ArmControllerRobai::setEndEffectorMode ( const ArmControllerRobai::EndEffectorMode& end_effector_mode )
{
    ROS_DEBUG("Setting new end effector mode");

	if (end_effector_mode_ == end_effector_mode) 
    	return true;
    
    if ( not setEndEffectorSet(end_effector_mode, getRobaiArmIndex()))
    {
     	ROS_ERROR("Could not set end effector mode");
        return false;
    }
    
    end_effector_mode_ = end_effector_mode;
    return true;
}

ArmControllerRobai::EndEffectorMode ArmControllerRobai::getEndEffectorMode()
{
    return end_effector_mode_;
}

int ArmControllerRobai::getRobaiEndEffectorMode()
{
    return static_cast<int>(end_effector_mode_);
}

bool ArmControllerRobai::resetEndEffectorSet()
{
    return setEndEffectorMode(end_effector_mode_);
}

bool ArmControllerRobai::disallowArmMovement()
{
    return setEndEffectorMode(NO_MOVEMENT_ARM_ALLOWED);
}

int ArmControllerRobai::getRobaiArmIndex()
{
    return 0;
}

int ArmControllerRobai::getRobaiGripperIndex()
{
    return 1;
}

bool ArmControllerRobai::setPositionControl()
{
    if (control_mode_ == POSITION) 
    	return true;

    // Control mode to set (0 for position control and 1 for velocity control)
    if (not setControlMode(0, getRobaiArmIndex()))
    {
        ROS_ERROR("Could not set control mode to velocity control");
        return false;
    }

    control_mode_ = POSITION;
    return true;
}

bool ArmControllerRobai::setVelocityControl()
{
    if (control_mode_ == VELOCITY) 
    	return true;

    // Control mode to set (0 for position control and 1 for velocity control)
    if (not setControlMode(1, getRobaiArmIndex()))
    {
        ROS_ERROR("Could not set control mode to velocity control");
        return false;
    }

    control_mode_ = VELOCITY;
    return true;
}

}; // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerRobai, arm_controller_base::ArmControllerBase);
