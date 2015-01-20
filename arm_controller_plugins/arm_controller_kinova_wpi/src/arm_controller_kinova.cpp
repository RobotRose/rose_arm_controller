/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Mathijs de Langen
*   Date  : 2015/01/07
*       - File created.
*
* Description:
*   This package describes the kinova interface for robot arms. This plugin uses
*   the WPI jaco package (http://wiki.ros.org/wpi_jaco)
* 
***********************************************************************************/
#include "arm_controller_kinova/arm_controller_kinova.hpp"

namespace arm_controller_plugins {

ArmControllerKinova::ArmControllerKinova()
	: n_("~kinova")
	, joint_states_initialized_(false)
	, emergency_(false)
	, gripper_width_(0.0)
{

}

ArmControllerKinova::~ArmControllerKinova()
{

}

bool ArmControllerKinova::initialize( const std::string name )
{
	name_ = name;
	ROS_INFO("Initializing arm <%s>", name.c_str());
	loadParameters();

	ros::NodeHandle n;

	// Register SMC client for gripper
	gripper_client_ = new GripperClient(arm_prefix_ + std::string("/fingers_controller"), true);
	move_it_client_ = new MoveItClient(moveit_server_name_, true);

	// Create all publishers
	arm_cartesian_command_publisher_	= n.advertise<wpi_jaco_msgs::CartesianCommand>(arm_prefix_ + std::string("/cartesian_cmd"), 1);
	arm_angular_command_publisher_ 		= n.advertise<wpi_jaco_msgs::AngularCommand>(arm_prefix_ + std::string("/angular_cmd"), 1);

	// Create all subscribers
	joint_state_sub_ 					= n.subscribe(arm_prefix_ + std::string("/joint_states"), 1, &ArmControllerKinova::CB_joint_state_received, this);

	// Create all service clients
	get_cartesian_position_client_ 		= n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>(arm_prefix_ + std::string("/get_cartesian_position"));

	return true;
}

bool ArmControllerKinova::close()
{
	//! @todo MdL [IMPL]: Implement this function.
	return false;
}

bool ArmControllerKinova::cancel()
{
	ROS_INFO("Cancel received");
	//! @todo MdL [IMPL]: Implement this function.
	return false;
}

bool ArmControllerKinova::emergencyStop()
{
	emergency_ = true;

	return cancel();
}

bool ArmControllerKinova::resetEmergencyStop()
{
	emergency_ = false;

	return true;
}

int ArmControllerKinova::getNumberOfJoints() 
{
	vector<double> joint_positions;
	if ( not getJointPositions(joint_positions))
		ROS_ERROR("Could not get number of joints");

	return joint_positions.size();
}

bool ArmControllerKinova::getEndEffectorPose(Pose& pose)
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

bool ArmControllerKinova::setEndEffectorPose(const Pose& end_effector_pose)
{	
	ROS_INFO("Setting end effector pose...");
	
	if (emergency_)
		return false;

	rose_moveit_controller::arm_goalGoal goal;
	goal.goal_pose.pose = end_effector_pose;

	move_it_client_->sendGoal(goal);
	move_it_client_->waitForResult(ros::Duration(0.0)); // infinite?

	return true;
}

bool ArmControllerKinova::getEndEffectorVelocity(Twist& twist)
{
	//! @todo MdL [IMPL]: Implement this function.
	return false;
}

bool ArmControllerKinova::setEndEffectorVelocity(const Twist& velocity)
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

bool ArmControllerKinova::getConstraints(Twist& twist)
{
	//! @todo MdL [IMPL]: Implement this function.	
	return false;
}

bool ArmControllerKinova::setConstraints(const Twist& constraint)
{
	if(emergency_)
		return false;

	//! @todo MdL [IMPL]: Implement this function.
	return false;
}

bool ArmControllerKinova::resetConstraints()
{
	//! @todo MdL [IMPL]: Implement this function.
	return false;
}

double ArmControllerKinova::getGripperWidth()
{
	return gripper_width_;

	//! @todo MdL: Test above, if correct: Remove below.
	vector<double> joint_positions;
	getJointPositions(joint_positions);

	vector<double> finger_positions;
	for ( int i = joint_positions.size() - nr_fingers_ ; i < joint_positions.size() ; i++ )
		finger_positions.push_back(joint_positions[i]);

	double percentage_total = 0;
	for ( const auto& finger_position : finger_positions )
		// Get percentage open Since fully closed is 6400, we take the inverse of the percentage
		percentage_total += 1.0 - finger_position/6400;
		
	percentage_total = percentage_total/nr_fingers_;

	return percentage_total * max_gripper_width_;
}

bool ArmControllerKinova::setGripperWidth(const double required_width)
{	
	if(emergency_)
		return false;

	control_msgs::GripperCommandGoal gripper_command;
	gripper_command.command.position = required_width;
	// gripper_command.command.max_effort = 10.0; If init 0, problem?

	gripper_client_->sendGoal(gripper_command);
	gripper_client_->waitForResult(ros::Duration(0.0));

	control_msgs::GripperCommandResultConstPtr result = gripper_client_->getResult();

	gripper_width_ = result->position;

	return result->reached_goal;

	// The two fingers on the MICO have a range of 
	// approximately 0 (fully open) to 6400 (fully closed). 
	// (http://wiki.ros.org/jaco_ros)

	// Limit to the min and max values (0, max_gripper_width_)
	std::min(std::max(required_width, 0.0), max_gripper_width_);

	// Calculate the percentage open
	double percentage_open = required_width/max_gripper_width_;

	vector<float> finger_angles;
	finger_angles.resize(nr_fingers_);
	for ( int i = 0 ; i < nr_fingers_ ; i++ )
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

bool ArmControllerKinova::getEndEffectorWrench(Wrench& wrench)
{
	ROS_ERROR("This arm does not support reading force/torque values");
	return false;
}

bool ArmControllerKinova::setEndEffectorWrench(const Wrench& Wrench)
{
	if(emergency_)
		return false;

	ROS_ERROR("This arm does not support force/torque control");
	return false;
}

bool ArmControllerKinova::getJointPositions(vector<double>& joint_positions)
{
	joint_states_mutex_.lock();
	
	joint_positions = joint_states_.position;

	joint_states_mutex_.unlock();

	return true;
}

bool ArmControllerKinova::setJointPositions(const vector<double>& joint_positions)
{
	return setAngularJointValues(joint_positions, true);
}

bool ArmControllerKinova::getJointVelocities(vector<double>& joint_velocities)
{
	joint_states_mutex_.lock();
	
	joint_velocities = joint_states_.velocity;

	joint_states_mutex_.unlock();

	return true;
}

bool ArmControllerKinova::setJointVelocities(const vector<double>& joint_velocities)
{
	return setAngularJointValues(joint_velocities, false);
}

bool ArmControllerKinova::getJointEfforts(vector<double>& joint_angular_forces)
{
	joint_states_mutex_.lock();
	
	joint_angular_forces = joint_states_.effort;

	joint_states_mutex_.unlock();

	return true;
}

bool ArmControllerKinova::setJointEfforts(const vector<double>& joint_angular_forces)
{
	//! @todo MdL [IMPL]: Implement this function.
	return false;
}

bool ArmControllerKinova::loadParameters()
{
    ROS_INFO("Loading kinova arm parameters for <%s>", name_.c_str());

    n_.param("/" + name_ + "_configuration/arm_prefix", arm_prefix_, std::string("kinova_arm"));
    n_.param("/" + name_ + "_configuration/max_gripper_width", max_gripper_width_, 0.15);
    n_.param("/" + name_ + "_configuration/nr_fingers", nr_fingers_, 3);
    n_.param("/" + name_ + "_configuration/moveit_server_name", moveit_server_name_, std::string("rose_moveit_controller"));

    ROS_INFO("Parameters loaded.");

    //! @todo MdL [IMPR]: Return is values are all correctly loaded.
    return true;
}

void ArmControllerKinova::CB_joint_state_received(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	joint_states_mutex_.lock();

	joint_states_initialized_ = true;
	joint_states_ 			  = *joint_state;

	joint_states_mutex_.unlock();
}

bool ArmControllerKinova::setAngularJointValues(const vector<double>& values, const bool& position)
{
	wpi_jaco_msgs::AngularCommand angular_cmd;
	angular_cmd.position 		= position;
	angular_cmd.armCommand 		= true;
	angular_cmd.repeat 			= true; 

	// Only arm joints
	if ( values.size() == NR_JOINTS )
	{
		angular_cmd.fingerCommand 	= false;
		std::copy ( values.begin(), values.begin() + NR_JOINTS, angular_cmd.joints.begin() );
	}
	// Arm and finger joints
	else if ( NR_JOINTS < values.size() and values.size() <= 9 )
	{
		angular_cmd.fingerCommand 	= true;	
		std::copy ( values.begin() + NR_JOINTS, values.end(), angular_cmd.fingers.begin() );
	}
	else
	{
		ROS_ERROR("Wrong number of joints given");
		return false;
	}

	arm_angular_command_publisher_.publish(angular_cmd);

	return true;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerKinova, arm_controller_base::ArmControllerBase);
