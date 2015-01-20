/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Mathijs de Langen
*   Date  : 2015/01/07
*       - File created.
*
* Description:
*   This package describes the mico interface for robot arms. This plugin uses
*   the WPI jaco package (http://wiki.ros.org/wpi_jaco)
* 
***********************************************************************************/
#include "arm_controller_jaco/arm_controller_jaco.hpp"

namespace arm_controller_plugins {

ArmControllerJaco::ArmControllerJaco()
	: ArmControllerKinova()
{
	
}

ArmControllerJaco::~ArmControllerJaco()
{

}

} // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerJaco, arm_controller_base::ArmControllerBase);
