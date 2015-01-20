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
#include "arm_controller_mico/arm_controller_mico.hpp"

namespace arm_controller_plugins {

ArmControllerMico::ArmControllerMico()
	: ArmControllerKinova()
{
	
}

ArmControllerMico::~ArmControllerMico()
{

}

} // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerMico, arm_controller_base::ArmControllerBase);
