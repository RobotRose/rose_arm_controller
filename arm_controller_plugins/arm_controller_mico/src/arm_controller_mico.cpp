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
#include "arm_controller_mico/arm_controller_mico.hpp"

namespace arm_controller_plugins {

ArmControllerMico::ArmControllerMico()
	: n_("~mico_arm")
	, joint_states_initialized_(false)
	, emergency_(false)
{
	ros::NodeHandle n;

	// Create all publishers
	arm_cartesian_command_publisher_	= n.advertise<wpi_jaco_msgs::CartesianCommand>(ARM_NAME + std::string("/cartesian_cmd"), 1);

	// Create all subscribers
	joint_state_sub_ 					= n.subscribe(ARM_NAME + std::string("/joint_states"), 1, &ArmControllerMico::CB_joint_state_received, this);

	// Create all service clients
	get_cartesian_position_client_ 		= n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>(ARM_NAME + std::string("/get_cartesian_position"));
}

ArmControllerMico::~ArmControllerMico()
{

}

bool ArmControllerMico::initialize( const std::string name )
{
	name_ = name;
	ROS_INFO("Initializing Mico arm...");
	//! @todo MdL: Implement.
	return false;
}

bool ArmControllerMico::close()
{
	//! @todo MdL: Implement.
	return false;
}

bool ArmControllerMico::cancel()
{
	ROS_INFO("Cancel received");
	//! @todo MdL: Implement.
	return false;
}

bool ArmControllerMico::emergencyStop()
{
	emergency_ = true;

	return cancel();
}

bool ArmControllerMico::resetEmergencyStop()
{
	emergency_ = false;

	return true;
}

int ArmControllerMico::getNumberOfJoints() 
{
	vector<double> joint_positions;
	if ( not getJointPositions(joint_positions))
		ROS_ERROR("Could not get number of joints");

	return joint_positions.size();
}

bool ArmControllerMico::getEndEffectorPose(Pose& pose)
{
	wpi_jaco_msgs::GetCartesianPosition get_cartesian_position_message;

	if ( not get_cartesian_position_client_.call(get_cartesian_position_message))
	{
		ROS_ERROR("Could not retrieve end effector position");
		return false;	
	}
	else
	{
		pose.position.x 	= get_cartesian_position_message.response.pos.linear.x;
		pose.position.y 	= get_cartesian_position_message.response.pos.linear.y;
		pose.position.z 	= get_cartesian_position_message.response.pos.linear.z; 
		pose.orientation 	= tf::createQuaternionMsgFromRollPitchYaw(
			get_cartesian_position_message.response.pos.angular.x,
			get_cartesian_position_message.response.pos.angular.y,
			get_cartesian_position_message.response.pos.angular.z
		);
		return true;
	}

	// This return statement should never be reached.
	return false;
}

bool ArmControllerMico::setEndEffectorPose(const Pose& end_effector_pose)
{	
	if (emergency_)
		return false;

	//! @todo MdL: Implement. MoveIt!
	return false;
}

bool ArmControllerMico::getEndEffectorVelocity(Twist& twist)
{
	//! @todo MdL: Implement.
	return false;
}

bool ArmControllerMico::setEndEffectorVelocity(const Twist& velocity)
{
	if (emergency_)
		return false;

	// wpi_jaco_msgs::CartesianCommand message definition
	// bool position             # true for a position command, false for a velocity command
	// bool armCommand           # true if this command includes arm joint inputs
	// bool fingerCommand        # true if this command includes finger inputs
	// bool repeat               # true if the command should be repeatedly sent over a short interval
	// geometry_msgs/Twist arm   # position (m, rad) or velocity (m/s, rad/s) arm command
	// float32[] fingers         # position (rad) or velocity (rad/s) finger command

	wpi_jaco_msgs::CartesianCommand cartesian_cmd;
	cartesian_cmd.position 		= false;
	cartesian_cmd.armCommand 	= true;
	cartesian_cmd.fingerCommand = false;
	cartesian_cmd.repeat 		= false; // What happens when you put this to true?
	cartesian_cmd.arm 			= velocity;

	arm_cartesian_command_publisher_.publish(cartesian_cmd);

	return true;
}

bool ArmControllerMico::getConstraints(Twist& twist)
{
	//! @todo MdL: Implement.	
	return false;
}

bool ArmControllerMico::setConstraints(const Twist& constraint)
{
	if(emergency_)
		return false;

	//! @todo MdL: Implement.
	return false;
}

bool ArmControllerMico::resetConstraints()
{
	//! @todo MdL: Implement.
	return false;
}

double ArmControllerMico::getGripperWidth()
{
	vector<double> joint_positions;
	getJointPositions(joint_positions);

	vector<double> finger_positions;
	for ( int i = joint_positions.size() - NR_FINGERS ; i < joint_positions.size() ; i++ )
		finger_positions.push_back(joint_positions[i]);

	double percentage_total = 0;
	for ( const auto& finger_position : finger_positions )
		// Get percentage open Since fully closed is 6400, we take the inverse of the percentage
		percentage_total += 1.0 - finger_position/6400;
		
	percentage_total = percentage_total/NR_FINGERS;

	return percentage_total * MAX_GRIPPER_WIDTH;
}

bool ArmControllerMico::setGripperWidth(const double required_width)
{	
	if(emergency_)
		return false;

	// The two fingers on the MICO have a range of 
	// approximately 0 (fully open) to 6400 (fully closed). 
	// (http://wiki.ros.org/jaco_ros)

	// Limit to the min and max values (0, MAX_GRIPPER_WIDTH)
	std::min(std::max(required_width, 0.0), MAX_GRIPPER_WIDTH);

	// Calculate the percentage open
	double percentage_open = required_width/MAX_GRIPPER_WIDTH;

	vector<float> finger_angles;
	finger_angles.resize(NR_FINGERS);
	for ( int i = 0 ; i < NR_FINGERS ; i++ )
		// Since fully closed is 6400, we take the inverse of the percentage open
		finger_angles[i] = (1.0 - percentage_open)*6400;

	// wpi_jaco_msgs::CartesianCommand message definition
	// bool position             # true for a position command, false for a velocity command
	// bool armCommand           # true if this command includes arm joint inputs
	// bool fingerCommand        # true if this command includes finger inputs
	// bool repeat               # true if the command should be repeatedly sent over a short interval
	// geometry_msgs/Twist arm   # position (m, rad) or velocity (m/s, rad/s) arm command
	// float32[] fingers         # position (rad) or velocity (rad/s) finger command

	wpi_jaco_msgs::CartesianCommand cartesian_cmd;
	cartesian_cmd.position 		= true;
	cartesian_cmd.armCommand 	= false;
	cartesian_cmd.fingerCommand = true;
	cartesian_cmd.repeat 		= false; 
	cartesian_cmd.fingers		= finger_angles;

	arm_cartesian_command_publisher_.publish(cartesian_cmd);

	return true;
}

bool ArmControllerMico::getEndEffectorWrench(Wrench& wrench)
{
	ROS_ERROR("Mico does not support reading force/torque values");
	return false;
}

bool ArmControllerMico::setEndEffectorWrench(const Wrench& Wrench)
{
	if(emergency_)
		return false;

	ROS_ERROR("Mico does not support force/torque control");
	return false;
}

bool ArmControllerMico::getJointPositions(vector<double>& joint_positions)
{
	joint_states_mutex_.lock();
	
	joint_positions = joint_states_.position;

	joint_states_mutex_.unlock();

	return true;
}

bool ArmControllerMico::getJointVelocities(vector<double>& joint_velocities)
{
	joint_states_mutex_.lock();
	
	joint_velocities = joint_states_.velocity;

	joint_states_mutex_.unlock();

	return true;
}

bool ArmControllerMico::getJointEfforts(vector<double>& joint_angular_forces)
{
	joint_states_mutex_.lock();
	
	joint_angular_forces = joint_states_.effort;

	joint_states_mutex_.unlock();

	return true;
}

void ArmControllerMico::CB_joint_state_received(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	joint_states_mutex_.lock();

	joint_states_initialized_ = true;
	joint_states_ 			  = *joint_state;

	joint_states_mutex_.unlock();
}

}; // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerMico, arm_controller_base::ArmControllerBase);
