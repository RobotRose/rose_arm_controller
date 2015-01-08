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
#ifndef ARM_CONTROLLER_MICO_HPP
#define ARM_CONTROLLER_MICO_HPP

#include <ros/ros.h>

#include <mutex>  
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

#include "arm_controller_base/arm_controller_base.hpp"

#include "wpi_jaco_msgs/JacoFK.h"
#include "wpi_jaco_msgs/CartesianCommand.h"
#include "wpi_jaco_msgs/GetCartesianPosition.h"

#define ARM_NAME            "jaco_arm"
#define MAX_GRIPPER_WIDTH   0.15 //[m]
#define NR_FINGERS          3 // Jaco has three, Mico uses this software and sets properties of the third finger to 0.0

namespace arm_controller_plugins {    

using geometry_msgs::Pose;
using geometry_msgs::Twist;
using geometry_msgs::Wrench;
using sensor_msgs::JointState;
using std::vector;

 /**
  * @brief Provides an interface to interact with each kind of arm.
  */
class ArmControllerMico : public arm_controller_base::ArmControllerBase {
  public:
    /**
    * @brief  Constructor
    */
    ArmControllerMico();

    /**
    * @brief  Destructor
    */
    ~ArmControllerMico();

    bool initialize( const std::string name = std::string() );
    
    /**
     * @brief When closing the arm controller
     * @details The arm has to shutdown
     * @return If it was successful
     */
    bool close();

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

    int getNumberOfJoints();

    /**
     * @brief Retrieves the position of the end effector.
     * @details A stamped pose of the gripper tip.
     * @return The end effector pose.
     */
    bool getEndEffectorPose(Pose& pose);

    /**
     * @brief Sets the end effector pose.
     * @details The arm will move to this position.
     * 
     * @param end_effector_pose The required end effector pose.
     * @return If the action was successful.
     */
    bool setEndEffectorPose(const Pose& end_effector_pose);

    /**
     * @brief Retrieves the end effector velocity.
     * @details Twist msg.
     * @return The end effector velocity in ROS twist message.
     */
    bool getEndEffectorVelocity(Twist& twist);

    /**
     * @brief Sets the end effector velicity.
     * @details The arm will try to move in the velocity required.
     * 
     * @param velocity The required velocity.
     * @return Is the action was successful.
     */
    bool setEndEffectorVelocity(const Twist& velocity);

    bool getConstraints(Twist& twist);

    bool setConstraints(const Twist& constraint);

    bool resetConstraints();

    double getGripperWidth();

    bool setGripperWidth(const double required_width);

    bool getEndEffectorWrench(Wrench& wrench);

    bool setEndEffectorWrench(const Wrench& Wrench);

    bool getJointPositions(vector<double>& joint_positions);

    bool getJointVelocities(vector<double>& joint_velocities);

    bool getJointEfforts(vector<double>& joint_angular_forces);

    // bool hasMoveItInterface();

  private:
    void CB_joint_state_received(const sensor_msgs::JointState::ConstPtr& joint_state);

    ros::NodeHandle     n_;
    std::string         name_;
    
    ros::Publisher      arm_cartesian_command_publisher_;
    ros::ServiceClient  get_cartesian_position_client_;

    bool                emergency_;

    std::mutex          joint_states_mutex_;
    JointState          joint_states_;
    ros::Subscriber     joint_state_sub_;
    bool                joint_states_initialized_;
};
};

#endif