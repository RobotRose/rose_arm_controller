/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/09/01
* 		- File created.
*
* Description:
*	This class provides visual servoing functionalities on the arms of the robot.
* 
***********************************************************************************/
#ifndef ARM_VISUAL_SERVOING_HPP
#define ARM_VISUAL_SERVOING_HPP

#include <iostream>
#include <limits>
#include <ros/ros.h>
#include <stdio.h>

#include <tf/transform_listener.h>

#include "rose_arm_controller_msgs/move_to_tfAction.h"
#include "rose_arm_controller_msgs/move_to_tfGoal.h"
#include "rose_arm_controller_msgs/move_to_tfFeedback.h"
#include "rose_arm_controller_msgs/move_to_tfResult.h"

#include "rose_arm_controller_msgs/set_velocityAction"
#include "rose_arm_controller_msgs/set_velocityGoal"
#include "rose_arm_controller_msgs/set_velocityFeedback"
#include "rose_arm_controller_msgs/set_velocityResult"

#include "server_multiple_client/server_multiple_client.hpp"

/**
 * This class provides visual servoing functionalities on the arms of the robot.
 */
class ArmVisualServoing 
{
  public:
  	/**
  	 * Server multiple client for this class.
  	 */
  	typedef ServerMultipleClient<rose_arm_controller_msgs::move_to_tfAction> SMC;

  	/**
  	 * Constructor
  	 */
  	ArmVisualServoing( std::string name, ros::NodeHandle n );

  	/**
  	 * Destructor
  	 */
  	~ArmVisualServoing();

  private:
	/**
	 * Add all arm as client of the SMC
	 */
	void addArmClients();
	
	/**
	 * Callback when goal has been received
	 * @param goal Goal
	 * @param smc  Sever multiple client object
	 */
	void CB_serverWork( const rose_arm_controller_msgs::move_to_tfGoalConstPtr& goal, SMC* smc );

	/**
	 * Callback when cancel has been received
	 * @param smc Server multiple client object
	 */
	void CB_serverCancel( SMC* smc );
	
	/**
	 * Callback when arm action was succesful
	 * @param state  End state
	 * @param result Result message
	 */
	void CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const rose_arm_controller_msgs::set_velocityResultPtr& result );

	/**
	 * Callback when arm action was \b not succesful
	 * @param state  End state
	 * @param result Result message
	 */
	void CB_armActionFail( const actionlib::SimpleClientGoalState& state, const rose_arm_controller_msgs::set_velocityResultPtr& result );

	/**
	 * Callback when the arm server is active
	 */
	void CB_armActionActive();

	/**
	 * Callback for feedback of the arm server
	 * @param feedback Feedback message
	 */
	void CB_armActionFeedback( const rose_arm_controller_msgs::set_velocityFeedbackConstPtr& feedback );

	/**
	 * Sends server action result
	 * @param succes If the action was successful
	 * @param result The result message (return code)
	 */
	void sendResult( const bool succes, const rose_arm_controller_msgs::set_velocityResultPtr& result );

	void stopMovement( const std::string arm );

	void sendArmSpeeds( const std::string arm, const double x, const double y, const double z );
	
	double distance(const double& x, const double& y);
	
  	ros::NodeHandle 	 n_;		//!< NodeHandle
  	std::string 		 name_;     //!< Name of the node
  	
	SMC*				  smc_;					 //!< Server multiple client
	tf::TransformListener tf_;					 //!< Transform listener

};

#endif // ARM_VISUAL_SERVOING_HPP