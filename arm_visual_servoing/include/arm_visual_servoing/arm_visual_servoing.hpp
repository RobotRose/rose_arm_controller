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

#include "rose_common/common.hpp"

#include "rose_arm_controller_msgs/move_to_tfAction.h"
#include "rose_arm_controller_msgs/move_to_tfGoal.h"
#include "rose_arm_controller_msgs/move_to_tfFeedback.h"
#include "rose_arm_controller_msgs/move_to_tfResult.h"

#include "rose_arm_controller_msgs/set_velocityAction.h"
#include "rose_arm_controller_msgs/set_velocityGoal.h"
#include "rose_arm_controller_msgs/set_velocityFeedback.h"
#include "rose_arm_controller_msgs/set_velocityResult.h"

#include "rose_geometry/geometry.hpp"
#include "rose_transformations/transformations.hpp"
#include "rose_conversions/conversions.hpp"

#include "server_multiple_client/server_multiple_client.hpp"

#define MAX_NR_CONSTRAINED_CONVERGENCES 	5
#define MAX_NR_FAILS						15
#define MAX_NR_NO_CONVERGENCES 				15
#define TRANFORM_TIMEOUT 					0.0167	//[s]
#define MAX_DIFFERENCE_BETWEEN_TFS			0.3 	//[s]

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
	 * Connect to the arm controller server
	 */
	void connectToArmController();
	
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
	void CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const rose_arm_controller_msgs::set_velocityResultConstPtr& result );
	/**
	 * Callback when arm action was \b not succesful
	 * @param state  End state
	 * @param result Result message
	 */
	void CB_armActionFail( const actionlib::SimpleClientGoalState& state, const rose_arm_controller_msgs::set_velocityResultConstPtr& result );

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
	void sendResult( const bool success );

	void stopMovement( const std::string arm_name );

	void sendArmSpeeds( const std::string arm_name, const double x, const double y, const double z,
													const double roll = 0.0, const double pitch = 0.0, const double yaw = 0.0 );
	
	/**
	 * @brief This function calculates the distance in a plane. 
	 * @details It is used for the constrained based visual servoing: When the distance in a certain place it too far from a point, it cannot move outside of the plance.
	 * 
	 * @param x First coordinate
	 * @param y Second coordinate
	 * 
	 * @return Distance in a place
	 */
	double distance(const double& x, const double& y);

  	ros::NodeHandle 	 n_;		//!< NodeHandle
  	std::string 		 name_;     //!< Name of the node

  	std::string 		 visual_servoing_server_name_;
  	
	SMC*				  smc_;					 //!< Server multiple client
	tf::TransformListener tf_;					 //!< Transform listener

	rose_arm_controller_msgs::set_velocityResultPtr result_;

};

#endif // ARM_VISUAL_SERVOING_HPP