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
#ifndef ARM_CONTROLLER_ROBAI_HPP
#define ARM_CONTROLLER_ROBAI_HPP

#include <pluginlib/class_list_macros.h>
#include "arm_controller_base/arm_controller_base.hpp"

namespace arm_controller_plugins {    

using geometry_msgs::PoseStamped;
using geometry_msgs::Twist;
using geometry_msgs::Wrench;

 /**
  * @brief Provides an interface to interact with each kind of arm.
  */
class ArmControllerRobai : public arm_controller_base::ArmControllerBase {
  public:
    /**
    * @brief  Constructor
    */

    ArmControllerRobai();
    /**
    * @brief  Destructor
    */
    ~ArmControllerRobai();

    bool initialize();

    /**
     * @brief   Cancels the current interaction of the arm. 
     * @details Cancelling only stopts the current interaction. The arm will respond again to new input.
     * @return  If the interaction was successfully cancelled.
     */
    bool cancel();

    /**
     * @brief   Stops the current interaction of the arm. 
     * @details The arm wil not respond to new inputs until resetEmergencyStop() has been called.
     * @return  If the interaction was successfully stopped.
     */
    bool emergencyStop();

    /**
     * @brief   Resets the emergency state to allow for new inputs.
     * @details Does not do anything if there is no emergency state.
     * @return  If the emergency state was successfully recoverd.
     */
    bool resetEmergencyStop();

    /**
     * @brief Retrieves the position of the end effector.
     * @details A stamped pose of the gripper tip.
     * @return The end effector pose.
     */
    PoseStamped getEndEffectorPose();

    /**
     * @brief Sets the end effector pose.
     * @details The arm will move to this position.
     * 
     * @param end_effector_pose The required end effector pose.
     * @return If the action was successful.
     */
    bool setEndEffectorPose(const PoseStamped& end_effector_pose);

    /**
     * @brief Retrieves the end effector velocity.
     * @details Twist msg.
     * @return The end effector velocity in ROS twist message.
     */
    Twist getEndEffectorVelocity();

    /**
     * @brief Sets the end effector velicity.
     * @details The arm will try to move in the velocity required.
     * 
     * @param velocity The required velocity.
     * @return Is the action was successful.
     */
    bool setEndEffectorVelocity(const Twist& velocity);

    Twist getContraints();

    bool setContraints(const Twist& contraint);

    bool resetContraints();

    double getGripperWidth();

    bool setGripperWidth(const double required_width);

    Wrench getEndEffectorWrench();

    bool setEndEffectorWrench(const Wrench& Wrench);

    // bool hasMoveItInterface();

  protected:
      
};
};

#endif