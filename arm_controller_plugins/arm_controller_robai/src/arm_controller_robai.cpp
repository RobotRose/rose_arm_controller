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
{

}

ArmControllerRobai::~ArmControllerRobai()
{

}

bool ArmControllerRobai::initialize()
{
	//! @todo MdL: Implement.
	return false;
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
	//! @todo MdL: Implement.
	return false;
}

Twist ArmControllerRobai::getEndEffectorVelocity()
{
	//! @todo MdL: Implement.
	Twist twist;
	return twist;
}

bool ArmControllerRobai::setEndEffectorVelocity(const Twist& velocity)
{
	//! @todo MdL: Implement.
	return false;
}

Twist ArmControllerRobai::getContraints()
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

bool ArmControllerRobai::setContraints(const Twist& constraint)
{
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

bool ArmControllerRobai::resetContraints()
{
	//! @todo MdL: Implement.
	return setEndEffectorMode(POINT_EE);
}

double ArmControllerRobai::getGripperWidth()
{
	//! @todo MdL: Implement.
	return 0.0;
}

bool ArmControllerRobai::setGripperWidth(const double required_width)
{
	return false;
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

bool ArmControllerRobai::setEndEffectorMode ( const ArmControllerRobai::EndEffectorMode& end_effector_mode )
{
    ROS_DEBUG("Setting new end effector mode");
    end_effector_mode_ = end_effector_mode;

    return setEndEffectorSet(end_effector_mode, 0);
}

ArmControllerRobai::EndEffectorMode ArmControllerRobai::getEndEffectorMode()
{
    return end_effector_mode_;
}

int ArmControllerRobai::getCurrentRobaiEndEffectorMode()
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

void ArmControllerRobai::setPositionControl()
{
    if (control_mode_ == POSITION) 
    	return;

    // Control mode to set (0 for position control and 1 for velocity control)
    if (not setControlMode(0, getRobaiArmIndex()))
    {
        ROS_ERROR("Could not set control mode to velocity control");
        return;
    }

    control_mode_ = POSITION;
}

void ArmControllerRobai::setVelocityControl()
{
    if (control_mode_ == VELOCITY) 
    	return;

    // Control mode to set (0 for position control and 1 for velocity control)
    if (not setControlMode(1, getRobaiArmIndex()))
    {
        ROS_ERROR("Could not set control mode to velocity control");
        return;
    }

    control_mode_ = VELOCITY;
}

}; // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerRobai, arm_controller_base::ArmControllerBase);
