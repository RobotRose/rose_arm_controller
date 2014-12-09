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
	return false;
}

bool ArmControllerRobai::cancel()
{
	return false;
}

bool ArmControllerRobai::emergencyStop()
{
	return false;
}

bool ArmControllerRobai::resetEmergencyStop()
{
	return false;
}

PoseStamped ArmControllerRobai::getEndEffectorPose()
{
	PoseStamped pose_stamped;
	return pose_stamped;
}

bool ArmControllerRobai::setEndEffectorPose(const PoseStamped& end_effector_pose)
{
	return false;
}

Twist ArmControllerRobai::getEndEffectorVelocity()
{
	Twist twist;
	return twist;
}

bool ArmControllerRobai::setEndEffectorVelocity(const Twist& velocity)
{
	return false;
}

Twist ArmControllerRobai::getContraints()
{
	Twist twist;
	return twist;
}

bool ArmControllerRobai::setContraints(const Twist& contraint)
{
	return false;
}

bool ArmControllerRobai::resetContraints()
{
	return false;
}

double ArmControllerRobai::getGripperWidth()
{
	return 0.0;
}

bool ArmControllerRobai::setGripperWidth(const double required_width)
{
	return false;
}

Wrench ArmControllerRobai::getEndEffectorWrench()
{
	Wrench wrench;
	return wrench;
}

bool ArmControllerRobai::setEndEffectorWrench(const Wrench& Wrench)
{
	return false;
}

}; // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerRobai, arm_controller_base::ArmControllerBase);
