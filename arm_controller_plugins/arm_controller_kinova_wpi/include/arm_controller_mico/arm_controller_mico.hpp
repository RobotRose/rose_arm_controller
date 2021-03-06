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
#ifndef ARM_CONTROLLER_MICO_WPI_HPP
#define ARM_CONTROLLER_MICO_WPI_HPP

#include "arm_controller_kinova/arm_controller_kinova.hpp"

namespace arm_controller_plugins {    

 /**
  * @brief Provides an interface to interact with each kind of arm.
  */
class ArmControllerMico : public ArmControllerKinova {
  public:
    /**
    * @brief  Constructor
    */
    ArmControllerMico();

    /**
    * @brief  Destructor
    */
    ~ArmControllerMico();

};

} // namepsace

#endif