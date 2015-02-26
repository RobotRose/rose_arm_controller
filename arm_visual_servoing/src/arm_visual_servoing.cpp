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

	//! @todo MdL [CONF]: Make this configurable.
	visual_servoing_server_name_ = "arm_controller/velocity";

	connectToArmController();

	smc_->startServer();
}

ArmVisualServoing::~ArmVisualServoing()
{

}

void ArmVisualServoing::connectToArmController()
{
	smc_->addClient<rose_arm_controller_msgs::set_velocityAction>(visual_servoing_server_name_, 
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
	double speed_scale 		= 1.5;//0.5; //! @todo MdL [CONF]: Make configurable (arm specific).
	double rotation_scale 	= 0.1; //! @todo MdL [CONF]: Make configurable (arm specific).

	// Check thresholds
	//! @todo MdL: Add thresholds.
	
	ROS_DEBUG_NAMED(ROS_NAME, "Visual servoing to item %s", frame_id.c_str() );

	int		nr_fails = 0;
	bool 	no_convergence = 0; 

	//! @todo MdL [CONF]: Fix magix numbers.
	while ( smc_->hasActiveGoal() and nr_fails < 15 and no_convergence < 15 )
	{
		// Error between tip and goal pose
		geometry_msgs::PoseStamped error;

		// Get error to tip in the frame of the required position
		if ( not rose_transformations::getFrameInFrame(tf_, frame_id, arm_name+"_observed_gripper_tip", error, 0.0167) )
		{
			//! @todo MdL: Do something smart (of something stupid, if that works) to see the marker again. Maybe random.
			ROS_ERROR("Cannot see gripper tip");
			nr_fails++;
			stopMovement(arm_name);
			continue; // Restart the while loop
		}

		//! @todo MdL [CONF]: Magic number; How old is allowed for the transform?
		if ( error.header.stamp < ros::Time::now() - ros::Duration(1.0))
		{
			ROS_ERROR("Error transform too old");
			ROS_DEBUG_NAMED(ROS_NAME, "Error time=%f", error.header.stamp.toSec());
			ROS_DEBUG_NAMED(ROS_NAME, "Current time=%f",ros::Time::now().toSec());
			nr_fails++;
			stopMovement(arm_name);
			continue; // Restart the while loop
		}

		// Get the origin of the error frame in the arm frame
		geometry_msgs::PoseStamped translation_between_frames;
		if ( not rose_transformations::getFrameInFrame( tf_, error.header.frame_id, arm_name, translation_between_frames, 0.0167 ))
		{
			ROS_ERROR("No transform found between %s and %s", error.header.frame_id.c_str(), arm_name.c_str());
			nr_fails++;
			stopMovement(arm_name);
			continue; // Restart the while loop
		}

		// Get the error position in the arm frame
		geometry_msgs::PoseStamped arm_pose_stamped;
		arm_pose_stamped.header.frame_id = error.header.frame_id;
		arm_pose_stamped.pose 			 = error.pose;
		if ( not rose_transformations::transformToFrameNow( tf_, arm_name, arm_pose_stamped, 0.0167 ))
		{
			ROS_ERROR("Could not transform to frame %s", arm_name.c_str());
			nr_fails++;
			stopMovement(arm_name);
			continue; // Restart the while loop
		}

		// Calculate the error distance (both in arm frame now)
		arm_pose_stamped.pose.position.x    -= translation_between_frames.pose.position.x;
		arm_pose_stamped.pose.position.y    -= translation_between_frames.pose.position.y;
		arm_pose_stamped.pose.position.z    -= translation_between_frames.pose.position.z; 

		// Calculate the difference in rotation
		tf::Quaternion translation_quat, arm_pose_quat;
		tf::quaternionMsgToTF(translation_between_frames.pose.orientation, translation_quat);
		tf::quaternionMsgToTF(arm_pose_stamped.pose.orientation, arm_pose_quat);

		arm_pose_quat = arm_pose_quat - translation_quat;

		tf::quaternionTFToMsg(arm_pose_quat, arm_pose_stamped.pose.orientation);

		geometry_msgs::Vector3 arm_pose_stamped_rpy = rose_conversions::quaternionToRPY(arm_pose_stamped.pose.orientation);

		ROS_DEBUG_NAMED(ROS_NAME, "Error = (%f,%f,%f):(%f,%f,%f)", 
			arm_pose_stamped.pose.position.x, 
			arm_pose_stamped.pose.position.y, 
			arm_pose_stamped.pose.position.z,
			arm_pose_stamped_rpy.x,
			arm_pose_stamped_rpy.y,
			arm_pose_stamped_rpy.z
		);

		// Check if we have reached the goal
		geometry_msgs::Pose zero;
		double error_distance = rose_geometry::distanceXYZ(zero, arm_pose_stamped.pose);
		
		ROS_DEBUG_NAMED(ROS_NAME, "Error distance: %f (minimum required = %f)", error_distance, max_distance);

		if ( error_distance < max_distance)
		{
			ROS_DEBUG_NAMED(ROS_NAME, "Goal reached");
			sendResult(true, result_);
			return;
		}

		// All is well, reset the number of fails
		nr_fails = 0; 

		// Store the values in easier variables
		double x 		= arm_pose_stamped.pose.position.x;
		double y 		= arm_pose_stamped.pose.position.y;
		double z 		= arm_pose_stamped.pose.position.z;

		double roll 	= arm_pose_stamped_rpy.x;
		double pitch 	= arm_pose_stamped_rpy.y;
		double yaw 		= arm_pose_stamped_rpy.z;

		double speed_x 		= x * speed_scale;
		double speed_y 		= y * speed_scale;
		double speed_z 		= z * speed_scale;

		double rotate_roll	= roll 	* rotation_scale;
		double rotate_pitch	= pitch * rotation_scale;
		double rotate_yaw 	= yaw 	* rotation_scale;
		
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

		sendArmSpeeds(arm_name, speed_x, speed_y, speed_z);
		// sendArmSpeeds(arm_name, speed_x, speed_y, speed_z, rotate_roll, rotate_pitch, rotate_yaw);
	}

	// While loop ended on fails, hence the goal has not been reached

	// Stop movement
	stopMovement(arm_name);

	ROS_ERROR("Goal NOT reached!");
	sendResult(false, result_);

}

void ArmVisualServoing::stopMovement( const std::string arm )
{
	ROS_DEBUG_NAMED(ROS_NAME, "Stop movement");
	sendArmSpeeds(arm,0,0,0);
}

void ArmVisualServoing::sendArmSpeeds( const std::string arm_name, const double x, const double y, const double z, 
																   const double roll, const double pitch, const double yaw )
{
	ROS_DEBUG_NAMED(ROS_NAME, "Sending arm movement (%f, %f, %f):(%f, %f, %f)", x, y, z, roll, pitch, yaw);
	geometry_msgs::TwistStamped arm_movement;

	//! @todo MdL [IMPR]: This needs a header.
	arm_movement.twist.linear.x = x;
	arm_movement.twist.linear.y = y;
	arm_movement.twist.linear.z = z;

	arm_movement.twist.angular.x = roll;
	arm_movement.twist.angular.y = pitch;
	arm_movement.twist.angular.z = yaw;

	rose_arm_controller_msgs::set_velocityGoal velocity_goal;
	velocity_goal.arm 				= arm_name;
	velocity_goal.required_velocity = arm_movement;

	smc_->sendGoal<rose_arm_controller_msgs::set_velocityAction>(velocity_goal, visual_servoing_server_name_);
}

void ArmVisualServoing::CB_serverCancel( SMC* smc )
{

}

void ArmVisualServoing::sendResult( const bool succes, const rose_arm_controller_msgs::set_velocityResultConstPtr& result )
{
	rose_arm_controller_msgs::move_to_tfResult move_result;
	// move_result.return_code = result->return_code;

	smc_->sendServerResult( succes, move_result );
}

double ArmVisualServoing::distance(const double& x, const double& y)
{
	// ROS_DEBUG_NAMED(ROS_NAME, "Distance for %f, %f : %f", x, y, sqrt(x*x+y*y));
	return sqrt(x*x+y*y);
}