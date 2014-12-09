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
	//! @todo MdL: Implement.
	Twist twist;
	return twist;
}

bool ArmControllerRobai::setContraints(const Twist& contraint)
{
	//! @todo MdL: Implement.
	return false;
}

bool ArmControllerRobai::resetContraints()
{
	//! @todo MdL: Implement.
	return false;
}

double ArmControllerRobai::getGripperWidth()
{
	//! @todo MdL: Implement.
	return 0.0;
}

bool ArmControllerRobai::setGripperWidth(const double required_width)
{
	//! @todo MdL: Implement.
	return false;
}

Wrench ArmControllerRobai::getEndEffectorWrench()
{
	//! @todo MdL: Implement.
	Wrench wrench;
	return wrench;
}

bool ArmControllerRobai::setEndEffectorWrench(const Wrench& Wrench)
{
	//! @todo MdL: Implement.
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

}; // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerRobai, arm_controller_base::ArmControllerBase);
