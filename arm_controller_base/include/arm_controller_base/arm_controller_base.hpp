/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Mathijs de Langen
*   Date  : 2014/12/05
*       - File created.
*
* Description:
*   This package describes the common interface for robot arms.
* 
***********************************************************************************/
#ifndef ARM_CONTROLLER_BASE_HPP
#define ARM_CONTROLLER_BASE_HPP

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace arm_controller_base {

using geometry_msgs::PoseStamped;
using geometry_msgs::Twist;
using geometry_msgs::Wrench;
 /**
  * @brief Provides an interface to interact with each kind of arm.
  */
class ArmControllerBase{
  public:
    
    /**
     * @brief   Cancels the current interaction of the arm. 
     * @details Cancelling only stopts the current interaction. The arm will respond again to new input.
     * @return  If the interaction was successfully cancelled.
     */
    virtual bool cancel() = 0;

    /**
     * @brief   Stops the current interaction of the arm. 
     * @details The arm wil not respond to new inputs until resetEmergencyStop() has been called.
     * @return  If the interaction was successfully stopped.
     */
    virtual bool emergencyStop() = 0;

    /**
     * @brief   Resets the emergency state to allow for new inputs.
     * @details Does not do anything if there is no emergency state.
     * @return  If the emergency state was successfully recoverd.
     */
    virtual bool resetEmergencyStop() = 0;

    virtual PoseStamped getEndEffectorPose() = 0;

    virtual bool setEndEffectorPose(const PoseStamped& end_effector_pose) = 0;

    virtual Twist getEndEffectorVelocity() = 0;

    virtual bool setEndEffectorVelocity(const Twist& velocity) = 0;

    virtual bool setContraint(const Twist& contraint) = 0;

    virtual bool setGripperWidth(const double required_width) = 0; // required_width in [m]

    virtual double getGripperWidth() = 0; // required_width in [m]

    virtual bool setEndEffectorWrench(const Wrench& Wrench) = 0;

    virtual Wrench getEndEffectorWrench() = 0;

    /**
    * @brief  Virtual destructor for the interface
    */
    virtual ~ArmControllerBase(){}

  protected:
      ArmControllerBase(){}
};
};

#endif