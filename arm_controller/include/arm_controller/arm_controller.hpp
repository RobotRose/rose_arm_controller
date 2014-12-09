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

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "arm_controller_base/arm_controller_base.hpp"

#include "rose20_common/common.hpp"
// #include "rose20_common/server_multiple_client/server_multiple_client.hpp"

// #include "action_result_message.hpp"

#include "rose_watchdogs/watchdog.hpp"

namespace arm_controller_core {

// #define MAX_ERROR 0.10        // in meters
// #define MAX_WAITING_TIME 4.5  // in seconds
#define VELOCITY_TIMEOUT 0.75  // seconds

using geometry_msgs::Pose;
using geometry_msgs::Vector3;
using geometry_msgs::Twist;
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

    /**
     * Constructor
     * @param name Node name
     * @param n Node handle
     */
    ArmController(std::string name, ros::NodeHandle n);

    /**
     * Destructor
     */
    ~ArmController();

    // ServerMultipleClient<arm_controller::manipulateAction>             set_position_smc_;
    // ServerMultipleClient<arm_controller::manipulateAction>             set_velocity_smc_;
    // ServerMultipleClient<arm_controller::manipulateAction>             set_gripper_width_smc_;

  private:
    std::string         name_;
    ros::NodeHandle     n_;

    pluginlib::ClassLoader<arm_controller_base::ArmControllerBase> arm_controller_plugin_loader_;
    
    // rose::Watchdog      velocity_watchdog_;
}; // ArmController
}; //namespace
#endif  // ARM_CONTROLLER_HPP
