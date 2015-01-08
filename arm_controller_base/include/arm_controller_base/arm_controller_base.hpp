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

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>

namespace arm_controller_base {

using geometry_msgs::PoseStamped;
using geometry_msgs::Pose;
using geometry_msgs::Twist;
using geometry_msgs::Wrench;
using sensor_msgs::JointState;
using std::vector;

 /**
  * @brief Provides an interface to interact with each kind of arm.
  */
class ArmControllerBase {
  public:

    /**
     * @brief Initializes all that is needed to control the arm
     * @details For instance: A Robai arm should connect to a certain ip address.
     * @return If initialization was successful.
     */
    virtual bool initialize( const std::string name = std::string() ) = 0;

    /**
     * @brief When closing the arm controller
     * @details The arm has to shutdown
     * @return If it was successful
     */
    virtual bool close() = 0;

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

    virtual int getNumberOfJoints() = 0;

    /**
     * @brief Retrieves the position of the end effector.
     * @details A stamped pose of the gripper tip.
     * @return The end effector pose.
     */
    virtual bool getEndEffectorPose(Pose&) = 0;

    /**
     * @brief Sets the end effector pose.
     * @details The arm will move to this position.
     * 
     * @param end_effector_pose The required end effector pose.
     * @return If the action was successful.
     */
    virtual bool setEndEffectorPose(const Pose&) = 0;

    /**
     * @brief Retrieves the end effector velocity.
     * @details Twist msg.
     * @return The end effector velocity in ROS twist message.
     */
    virtual bool getEndEffectorVelocity(Twist&) = 0;

    /**
     * @brief Sets the end effector velicity.
     * @details The arm will try to move in the velocity required.
     * 
     * @param velocity The required velocity.
     * @return Is the action was successful.
     */
    virtual bool setEndEffectorVelocity(const Twist&) = 0;

    virtual bool getConstraints(Twist&) = 0;

    virtual bool setConstraints(const Twist&) = 0;

    virtual bool resetConstraints() = 0;

    virtual double getGripperWidth() = 0; // required_width in [m]

    virtual bool setGripperWidth(const double required_width) = 0; // required_width in [m]

    virtual bool getEndEffectorWrench(Wrench&) = 0;

    virtual bool setEndEffectorWrench(const Wrench&) = 0;

    virtual bool getJointPositions(vector<double>&) = 0;

    virtual bool getJointVelocities(vector<double>&) = 0;

    virtual bool getJointEfforts(vector<double>&) = 0;

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