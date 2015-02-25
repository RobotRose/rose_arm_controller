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
#include "arm_visual_servoing/arm_visual_servoing.hpp"

ArmVisualServoing::ArmVisualServoing( std::string name, ros::NodeHandle n )
	: n_ ( n )
	, name_ ( name )
{
	smc_ = new SMC(n_, name_, boost::bind(&ArmVisualServoing::CB_serverWork, this, _1, _2),
						      boost::bind(&ArmVisualServoing::CB_serverCancel, this, _1));

	connectToArmController();

	smc_->startServer();
}

ArmVisualServoing::~ArmVisualServoing()
{

}

void ArmVisualServoing::connectToArmController()
{
	smc_->addClient<rose_arm_controller_msgs::set_velocityAction>("arm_controller", 
			boost::bind(&ArmVisualServoing::CB_armActionSuccess, this, _1, _2),
			boost::bind(&ArmVisualServoing::CB_armActionFail, this, _1, _2),
			boost::bind(&ArmVisualServoing::CB_armActionActive, this),
	   		boost::bind(&ArmVisualServoing::CB_armActionFeedback, this, _1)
	);
}

void ArmVisualServoing::CB_armActionSuccess( const actionlib::SimpleClientGoalState& state, const rose_arm_controller_msgs::set_velocityResultConstPtr& result )
{
    //ROS_INFO("ArmVisualServoing::CB_armActionSuccess");
    // result_ = result;
}

void ArmVisualServoing::CB_armActionFail( const actionlib::SimpleClientGoalState& state, const rose_arm_controller_msgs::set_velocityResultConstPtr& result )
{
  	//ROS_INFO("ArmVisualServoing::CB_armActionFail");
  	// result_ = result;
}

void ArmVisualServoing::CB_armActionActive()
{
	//ROS_INFO("ArmVisualServoing::CB_armActionActive");
}

void ArmVisualServoing::CB_armActionFeedback( const rose_arm_controller_msgs::set_velocityFeedbackConstPtr& feedback )
{
	//ROS_INFO("ArmVisualServoing::CB_armActionFeedback");
}

void ArmVisualServoing::CB_serverWork( const rose_arm_controller_msgs::move_to_tfGoalConstPtr& goal, SMC* smc )
{
	// Get arm anme
	std::string arm_name = goal->arm_name;

	// Get frame_id
	std::string frame_id = goal->tf_name;

	//! @todo MdL: Frame convertion?.
	double limit_dist_xy = ( goal->max_xy_error == 0.0 ? std::numeric_limits<double>::max() : goal->max_xy_error); 
	double limit_dist_xz = ( goal->max_xz_error == 0.0 ? std::numeric_limits<double>::max() : goal->max_xz_error);
	double limit_dist_yz = ( goal->max_yz_error == 0.0 ? std::numeric_limits<double>::max() : goal->max_yz_error);

	ROS_DEBUG_NAMED(ROS_NAME, "Found the following limits xy: %f xz: %f yz: %f", limit_dist_xy, limit_dist_xz, limit_dist_yz );

	// Get max distance to goal
	double max_distance 	= 0.005;
	double max_speed 		= 0.01;
	double closest_distance = std::numeric_limits<double>::max();
	double speed_scale 		= 0.5;

	// Check thresholds
	//! @todo MdL: Add thresholds.
	
	ROS_DEBUG_NAMED(ROS_NAME, "Visual servoing to item %s", frame_id.c_str() );

	int		nr_fails = 0;
	bool 	reached_goal = false;
	bool 	no_convergence = 0; 

	double speed_x = 0;
	double speed_y = 0;
	double speed_z = 0;

	while ( smc_->hasActiveGoal() and not reached_goal and nr_fails < 15 and no_convergence < 15 )
	{
		geometry_msgs::PoseStamped error;
		bool found_error = true;
		// Get error to tip in correct frame
		if ( not rose_transformations::getFrameInFrame(tf_, frame_id, arm_name+"_observed_gripper_tip", error, 0.0167) )
		{
			//! @todo MdL: Do something smart (of something stupid, if that works) to see the marker again. Maybe random.
			ROS_ERROR("Cannot see gripper tip error");
			nr_fails++;
			found_error = false;
			stopMovement(arm_name);
		}
		else
			nr_fails = 0;

		geometry_msgs::PoseStamped translation_between_frames;
		if (found_error)
		{
			// Error to arms frame
			if ( not rose_transformations::getFrameInFrame( tf_, error.header.frame_id, arm_name, translation_between_frames, 0.0167 ))
			{
				ROS_ERROR("No transform found between %s and %s", error.header.frame_id.c_str(), arm_name.c_str());
				found_error = false;
				stopMovement(arm_name);
			}
		}

		geometry_msgs::PoseStamped arm_pose_stamped;
		if (found_error)
		{
			arm_pose_stamped.header.frame_id = error.header.frame_id;
			arm_pose_stamped.pose 			 = error.pose;
			if ( not rose_transformations::transformToFrameNow( tf_, arm_name, arm_pose_stamped, 0.0167 ))
			{
				ROS_ERROR("Could not transform to frame %s", arm_name.c_str());
				found_error = false;
				stopMovement(arm);
			}
		}

		if (found_error)
		{
			arm_pose_stamped.pose.position.x -= translation_between_frames.pose.position.x;
			arm_pose_stamped.pose.position.y -= translation_between_frames.pose.position.y;
			arm_pose_stamped.pose.position.z -= translation_between_frames.pose.position.z; 

			double x = arm_pose_stamped.pose.position.x;
			double y = arm_pose_stamped.pose.position.y;
			double z = arm_pose_stamped.pose.position.z;

			Pose zero;
			double error_distance = rose_geometry::distanceXYZ(zero, arm_pose_stamped.pose);
			ROS_DEBUG_NAMED(ROS_NAME, "Found distance to %s: %f", frame_id.c_str(), error_distance);
			if ( error_distance < max_distance)
				reached_goal = true;

			ROS_DEBUG_NAMED(ROS_NAME, "Error: (%f,%f,%f)", arm_pose_stamped.pose.position.x, arm_pose_stamped.pose.position.y, arm_pose_stamped.pose.position.z);

			speed_x = x * speed_scale;
			speed_y = y * speed_scale;
			speed_z = z * speed_scale;
			
			if ( error_distance < closest_distance )
			{
				ROS_DEBUG_NAMED(ROS_NAME, "Converging.. New error: %f; old error: %f", error_distance, closest_distance);
				no_convergence = 0;
			}
			else 
			{
				ROS_DEBUG_NAMED(ROS_NAME, "Not converging.. New error: %f; old error: %f", error_distance, closest_distance);
				no_convergence++;
			}

			closest_distance = std::min(closest_distance, error_distance);

			//limit speeds, if needed
			if (distance(x,y) > limit_dist_xy and no_convergence < 5)
				speed_z = 0.0;
			if (distance(x,z) > limit_dist_xz and no_convergence < 5)
				speed_y = 0.0;
			if (distance(y,z) > limit_dist_yz and no_convergence < 5)
				speed_x = 0.0;

			if (speed_x == 0.0 and speed_y == 0.0 and speed_z == 0.0)
			{
				ROS_ERROR("Limits are not properly set (speeds are all zero)");
				nr_fails = 30;
			}

			sendArmSpeeds(arm, speed_x, speed_y, speed_z);
		}
	}

	// Stop movement
	stopMovement(arm);

	if ( not reached_goal )
	{
		ROS_INFO_NAMED(ROS_NAME, "Goal reached!");
		sendResult(false, result_); //! @todo MdL: Not succes of course .
	}
	else
	{
		ROS_INFO_NAMED(ROS_NAME, "Goal NOT reached!");
		sendResult(true, result_);
	}
}

void ArmVisualServoing::stopMovement( const std::string arm )
{
	ROS_DEBUG_NAMED(ROS_NAME, "Stop movement");
	sendArmSpeeds(arm,0,0,0);
}

void ArmVisualServoing::sendArmSpeeds( const std::string arm, const double x, const double y, const double z )
{
	ROS_DEBUG_NAMED(ROS_NAME, "Sending arm movement (%f, %f, %f)", x, y, z);
	geometry_msgs::Twist arm_movement;

	arm_movement.linear.x = x;
	arm_movement.linear.y = y;
	arm_movement.linear.z = z;

	rose_arm_controller_msgs::set_velocityGoal velocity_goal;
	velocity_goal.arm 				= arm_name;
	velocity_goal.required_velocity = arm_movement;

	smc_->sendGoal<rose_arm_controller_msgs::set_velocityAction>(velocity_goal, "arm_controller");
}

void ArmVisualServoing::CB_serverCancel( SMC* smc )
{

}

void ArmVisualServoing::sendResult( const bool succes, const arm_controller::set_velocityResultConstPtr& result )
{
	rose_arm_controller_msgs::move_to_tfResult move_result;
	// move_result.return_code = result->return_code;

	smc_->sendServerResult( succes, move_result );
}

double ArmVisualServoing::distance(const double& x, const double& y)
{
	ROS_DEBUG_NAMED(ROS_NAME, "Distance for %f, %f : %f", x, y, sqrt(x*x+y*y));
	return sqrt(x*x+y*y);
}