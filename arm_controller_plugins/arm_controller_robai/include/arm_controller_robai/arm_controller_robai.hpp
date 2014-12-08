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
#include <rose_arm_controller/arm_controller_base.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)

namespace arm_controller_plugins {

using geometry_msgs::PoseStamped;
using geometry_msgs::Twist;
using geometry_msgs::Wrench;

 /**
  * @brief Provides an interface to interact with each kind of arm.
  */
class ArmControllerRobai{
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

    /**
     * @brief Retrieves the position of the end effector.
     * @details A stamped pose of the gripper tip.
     * @return The end effector pose.
     */
    virtual PoseStamped getEndEffectorPose() = 0;

    /**
     * @brief Sets the end effector pose.
     * @details The arm will move to this position.
     * 
     * @param end_effector_pose The required end effector pose.
     * @return If the action was successful.
     */
    virtual bool setEndEffectorPose(const PoseStamped& end_effector_pose) = 0;

    /**
     * @brief Retrieves the end effector velocity.
     * @details Twist msg.
     * @return The end effector velocity in ROS twist message.
     */
    virtual Twist getEndEffectorVelocity() = 0;

    /**
     * @brief Sets the end effector velicity.
     * @details The arm will try to move in the velocity required.
     * 
     * @param velocity The required velocity.
     * @return Is the action was successful.
     */
    virtual bool setEndEffectorVelocity(const Twist& velocity) = 0;

    virtual Twist getContraints() = 0;

    virtual bool setContraints(const Twist& contraint) = 0;

    virtual bool resetContraints() = 0;

    virtual double getGripperWidth() = 0; // required_width in [m]

    virtual bool setGripperWidth(const double required_width) = 0; // required_width in [m]

    virtual Wrench getEndEffectorWrench() = 0;

    virtual bool setEndEffectorWrench(const Wrench& Wrench) = 0;

    // virtual bool hasMoveItInterface() = 0;

    /**
    * @brief  Virtual destructor for the interface
    */
    virtual ~ArmControllerBase(){}

  protected:
      ArmControllerBase(){}
};
};

#endif