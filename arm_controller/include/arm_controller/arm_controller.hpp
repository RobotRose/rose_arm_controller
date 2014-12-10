/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/12/08
* 		- File created.
*
* Description:
*	The generic arm controller
* 
***********************************************************************************/
#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <iostream>
#include <stdio.h>

#include <boost/lexical_cast.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "arm_controller_base/arm_controller_base.hpp"

#include "rose20_common/common.hpp"
#include "rose20_common/server_multiple_client/server_multiple_client.hpp"

#include "rose_arm_controller_msgs/set_positionAction.h"
#include "rose_arm_controller_msgs/set_positionGoal.h"
#include "rose_arm_controller_msgs/set_velocityAction.h"
#include "rose_arm_controller_msgs/set_velocityGoal.h"
#include "rose_arm_controller_msgs/set_gripper_widthAction.h"
#include "rose_arm_controller_msgs/set_gripper_widthGoal.h"

// #include "action_result_message.hpp"

#include "rose_watchdogs/watchdog.hpp"
#include "shared_variables/shared_variable.hpp"

namespace arm_controller_core {

// #define MAX_ERROR 0.10        // in meters
// #define MAX_WAITING_TIME 4.5  // in seconds
#define VELOCITY_TIMEOUT 0.75  // seconds

using geometry_msgs::Pose;
using geometry_msgs::Vector3;
using geometry_msgs::Twist;
using shared_variables::SharedVariable;
using std::find;
using std::map;
using std::vector;

/**
 * This is the basic arm controller. It provides an interface to use a number of
 * arms in a generic way.
 */
class ArmController
{
  public:
    typedef ServerMultipleClient<rose_arm_controller_msgs::set_positionAction>      SMC_position;
    typedef ServerMultipleClient<rose_arm_controller_msgs::set_velocityAction>      SMC_velocity;
    typedef ServerMultipleClient<rose_arm_controller_msgs::set_gripper_widthAction> SMC_gripper;

    /**
     * Constructor
     * @param name  Node name
     * @param n     Node handle
     */
    ArmController(std::string name, ros::NodeHandle n);

    /**
     * Destructor
     */
    ~ArmController();

    std::vector<std::string> getArms();

  private:
    // All startup procedures
    void createSMCs();
    void loadArmParameters();
    void loadArmPlugins();
    void initializeArmControllers();
    void registerSharedVariables();
    void testArmMovement();

    // Closing procedure
    void closeAllArmControllers();

    // Arm movements
    bool stopArmMovement(const boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller);
    void testMovementGrippers();

    void CB_receivePositionGoal(const rose_arm_controller_msgs::set_positionGoalConstPtr& goal, SMC_position* smc);
    void CB_receivePositionCancel(SMC_position* smc);
    void CB_receiveVelocityGoal(const rose_arm_controller_msgs::set_velocityGoalConstPtr& goal, SMC_velocity* smc);
    void CB_receiveVelocityCancel(SMC_velocity* smc);
    void CB_receiveGripperGoal(const rose_arm_controller_msgs::set_gripper_widthGoalConstPtr& goal, SMC_gripper* smc);
    void CB_receiveGripperCancel(SMC_gripper* smc);

    void CB_cancelVelocityForArms();
    void CB_emergency(const bool& emergency);

    std::string         name_;
    ros::NodeHandle     n_;

    int                                     nr_of_arms_;
    std::map<std::string, std::string>      arm_plugins_;

    SMC_position*  set_position_smc_;
    SMC_velocity*  set_velocity_smc_;
    SMC_gripper*   set_gripper_width_smc_;

    pluginlib::ClassLoader<arm_controller_base::ArmControllerBase>          arm_controller_plugin_loader_;
    std::map<std::string, boost::shared_ptr<arm_controller_base::ArmControllerBase>>  arm_controllers_;

    SharedVariable<bool>    sh_emergency_;      //!< Shared variable
    rose::Watchdog          velocity_watchdog_; //!< Watchdog for velocities

}; // ArmController
}; //namespace
#endif  // ARM_CONTROLLER_HPP
