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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

#include "arm_controller_base/arm_controller_base.hpp"

//robai
#include "arm_controller_robai/robai_manipulation_action.hpp"
#include "remoteCommand/ecRemoteCommand.h"
#include "remoteCommandClientPlugin/remoteCommandPlugin.h"

namespace arm_controller_plugins {    

using geometry_msgs::Pose;
using geometry_msgs::Twist;
using geometry_msgs::Wrench;
using sensor_msgs::JointState;
using std::vector;

using namespace Ec;
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

    bool getJointPositions(vector<double>&);

    bool setJointPositions(const vector<double>&);

    bool getJointVelocities(vector<double>&);

    bool setJointVelocities(const vector<double>&);

    bool getJointEfforts(vector<double>&);

    bool setJointEfforts(const vector<double>&);

    // bool hasMoveItInterface();

  private:
    // Robai modes, one-to-one mapping with the cyton file
    enum EndEffectorMode
    {
        POINT_EE = 0,   //!< End effector is free to move in every direction
        FREE_SPIN = 1,  //!< End effector needs to be horizontally, but can move
                        // in the horizontal plane
        FRAME_EE = 2,   //!< End effector must keep its end effector in the same
                        // orientation
        NO_MOVEMENT_ARM_ALLOWED = 3,
        count           //!< To count all possible end effectors (used in loops for
                        // instance)
    };

    // Robai manipulators, one-to-one mapping with the cyton file
    enum Manipulators
    {
        ARM = 0,    //!< Arm manipulator
        BODY = 1,   //!< Body manipulator
        TABLE = 2,  //!< Table manipulator
        BOX = 3,    //!< Box manipulator
        last
    };

    enum ControlMode
    {
        POSITION,
        VELOCITY,
    };

    bool connectToArms(const std::string ip = "127.0.0.1");
    bool loadParameters();

    EndEffectorMode getEndEffectorMode();
    bool setEndEffectorMode ( const EndEffectorMode& end_effector_mode, const bool save_state = true );

    Pose getCorrectedEndEffectorPose(const Pose& pose);
    int getRobaiArmIndex();
    int getRobaiGripperIndex();
    int getRobaiEndEffectorMode();

    bool resetEndEffectorSet();
    bool disallowArmMovement();

    bool setPositionControl();
    bool setVelocityControl();

    ros::NodeHandle n_;
    std::string     name_;
    
    bool emergency_;

    // parameters
    double  gripper_tip_correction_parameter_;
    int     max_manipulation_tries_parameter_;
    bool    enable_path_planning_function_parameter_;

    arm_controller_robai::RobaiManipulationAction manipulation_action_manager_;

    EndEffectorMode end_effector_mode_;
    ControlMode     control_mode_;
      
};
};

#endif