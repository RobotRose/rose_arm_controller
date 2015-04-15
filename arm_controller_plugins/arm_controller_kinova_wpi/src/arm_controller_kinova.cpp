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
	, in_collision_(false)
{

}

ArmControllerKinova::~ArmControllerKinova()
{

}

bool ArmControllerKinova::initialize( const std::string name )
{
	ros::NodeHandle n;

	name_ = name;

	ROS_INFO("Initializing arm <%s>", name.c_str());
	loadParameters();
	loadMoveitConfiguration();

	// Register actionlib client to the wpi_jaco driver for the gripper
	gripper_client_ = new GripperClient(arm_prefix_ + std::string("/fingers_controller/gripper"), true);
	gripper_client_->waitForServer();

	// Create all publishers to the wpi_jaco driver
	arm_cartesian_command_publisher_	= n.advertise<wpi_jaco_msgs::CartesianCommand>(arm_prefix_ + std::string("/cartesian_cmd"), 1);
	arm_angular_command_publisher_ 		= n.advertise<wpi_jaco_msgs::AngularCommand>(arm_prefix_ + std::string("/angular_cmd"), 1);

	// Create all subscribers to the wpi_jaco driver
	//! @todo MdL [HACK]: fix this back, is was for testing.
	joint_state_sub_ 					= n.subscribe(arm_prefix_ + std::string("/joint_states"), 1, &ArmControllerKinova::CB_joint_state_received, this);
	// joint_state_sub_ 					= n.subscribe(arm_prefix_ + std::string("/joint_states"), 1, &ArmControllerKinova::CB_joint_state_received, this);

	// Create all service clients to the wpi_jaco driver
	get_cartesian_position_client_ 		= n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>(arm_prefix_ + std::string("/get_cartesian_position"));

	// Create all timers
	// collision_check_timer_ 				= n.createTimer(ros::Duration(0.1), boost::bind(&ArmControllerKinova::updateCollisions, this));

	// For visualization
	visualization_pub_  				= n.advertise<visualization_msgs::Marker>(arm_prefix_ + std::string("/goal_pose"), 1);

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
	ROS_DEBUG("Setting end effector pose...");
	
	// Visualize goal pose
	showEndEffectorGoalPose(end_effector_pose);

	if (emergency_)
		return false;

	if (planning_scene_ == NULL)
	{
		ROS_ERROR_NAMED("path-planning", "No planning scene set");
		return false; 
	}

	if ( not updatePlanningScene() )
	{
		ROS_ERROR_NAMED("path-planning", "Could not update planning scene");
		return false;
	}


	robot_state::RobotState start_state(*move_group_->getCurrentState());
	move_group_->setStartState(start_state);

	//! @todo MdL [IMPR]: Configurable?.
	std::string  planner_plugin_name 	= "RRTkConfigDefault";
	double 		 planning_time 			= 0.5;
	double 	     goal_tolerance  		= 0.005;
	unsigned int num_planning_attempts 	= 3;

	// Timing planning and execution
	ros::Time timer = ros::Time::now();

	ROS_INFO("Computing plan");
	// Compute plan
	moveit::planning_interface::MoveGroup::Plan plan;
	move_group_->setPlannerId(planner_plugin_name);
	move_group_->setPlanningTime(planning_time);
	move_group_->setNumPlanningAttempts (num_planning_attempts);
	move_group_->setGoalTolerance(goal_tolerance);
	move_group_->setPoseTarget(end_effector_pose);

	if ( not move_group_->plan(plan) )
	{
		ROS_ERROR_NAMED("path-planning", "No plan found");
		return false; // Planning failed
	}

	ROS_INFO("Planning took %f seconds", (ros::Time::now() - timer).toSec() );

	timer = ros::Time::now();
	ROS_INFO("Executing plan");
	if ( not move_group_->execute(plan) )
	{
		ROS_ERROR_NAMED("path-planning", "Could not execute plan");
		return false; // Execution failed
	}

	ROS_INFO("Plan executed in %f seconds", (ros::Time::now() - timer).toSec() );

	return true;
}

bool ArmControllerKinova::getEndEffectorVelocity(Twist& twist)
{
	//! @todo MdL [IMPL]: Implement this function.
	return false;
}

bool ArmControllerKinova::setEndEffectorVelocity(const Twist& velocity)
{
	ROS_DEBUG("Setting end effector velocity...");
	if (emergency_)
		return false;

	// wpi_jaco_msgs::CartesianCommand message definition
	// bool position             # true for a position command, false for a velocity command
	// bool armCommand           # true if this command includes arm joint inputs
	// bool fingerCommand        # true if this command includes finger inputs
	// bool repeat               # true if the command should be repeatedly sent over a short interval, needed for vel control
	// geometry_msgs/Twist arm   # position (m, rad) or velocity (m/s, rad/s) arm command
	// float32[] fingers         # position (rad) or velocity (rad/s) finger command

	wpi_jaco_msgs::CartesianCommand cartesian_cmd;
	cartesian_cmd.position 		= false;
	cartesian_cmd.armCommand 	= true;
	cartesian_cmd.fingerCommand = false;
	cartesian_cmd.repeat 		= true; 
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
	ROS_DEBUG("Setting gripper width to %f", required_width);
	if(emergency_)
		return false;

	std::min(std::max(required_width, 0.0), max_gripper_width_);
	double percentage_open = required_width/max_gripper_width_;

	ROS_DEBUG("Gripper needs to be %f percentage open", percentage_open);

	double gripper_value;
	//! @todo MdL [IMPR]: Quick fix, when open value < closed vlue.
	if ( gripper_value_open_ > gripper_value_closed_)
		gripper_value = percentage_open * gripper_value_open_;
	else
	 	gripper_value = (1-percentage_open) * gripper_value_closed_;

	control_msgs::GripperCommandGoal gripper_command;
	gripper_command.command.position = gripper_value;
	// gripper_command.command.max_effort = 10.0; If init 0, problem?

	ROS_DEBUG("Sending gripper %f width to driver", gripper_value);
	gripper_client_->sendGoal(gripper_command);
	if ( not gripper_client_->waitForResult(ros::Duration(5.0)) )
		return false; //! @todo MdL [IMPR]: Better idea for time out?

	control_msgs::GripperCommandResultConstPtr result = gripper_client_->getResult();

	gripper_width_ = result->position;

	return result->reached_goal;

	// The two fingers on the MICO have a range of 
	// approximately 0 (fully open) to 6400 (fully closed). 
	// (http://wiki.ros.org/jaco_ros)

	// Limit to the min and max values (0, max_gripper_width_)
	
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
	std::lock_guard<std::mutex> lock(joint_states_mutex_);
	
	joint_positions = joint_states_.position;
	return true;
}

bool ArmControllerKinova::setJointPositions(const vector<double>& joint_positions)
{
	return setAngularJointValues(joint_positions, true);
}

bool ArmControllerKinova::getJointVelocities(vector<double>& joint_velocities)
{
	std::lock_guard<std::mutex> lock(joint_states_mutex_);
	
	joint_velocities = joint_states_.velocity;
	return true;
}

bool ArmControllerKinova::setJointVelocities(const vector<double>& joint_velocities)
{
	return setAngularJointValues(joint_velocities, false);
}

bool ArmControllerKinova::getJointEfforts(vector<double>& joint_angular_forces)
{
	std::lock_guard<std::mutex> lock(joint_states_mutex_);
	
	joint_angular_forces = joint_states_.effort;
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
    n_.param("/" + name_ + "_configuration/gripper_value_open", gripper_value_open_, 0.0);
    n_.param("/" + name_ + "_configuration/gripper_value_closed", gripper_value_closed_, 40.0);
    n_.param("/" + name_ + "_configuration/nr_fingers", nr_fingers_, 3);
    // n_.param("/" + name_ + "_configuration/moveit_server_name", moveit_server_name_, std::string("rose_moveit_controller"));

    ROS_INFO("Parameters loaded.");

    //! @todo MdL [IMPR]: Return if values are all correctly loaded.
    return true;
}

bool ArmControllerKinova::loadMoveitConfiguration()
{
	ROS_INFO("Loading MoveIt! configuration for <%s>", name_.c_str());

	// Initialize service client
	planning_scene_service_client_ 		= n_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

	ROS_DEBUG("Loading planning scene");
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr 	kinematic_model;

	// Make sure MoveIt! is running
	if (!planning_scene_service_client_.exists())
	{
	  ROS_DEBUG("Waiting for service %s to exist.", planning_scene_service_client_.getService().c_str());
	  planning_scene_service_client_.waitForExistence(ros::Duration(60.0));
	}

	int nr_fails = 0;
	while (not kinematic_model && nr_fails < 100)
	{
		ROS_DEBUG("Loading..");
		kinematic_model = robot_model_loader.getModel();		
		nr_fails++;
	}
	if ( not kinematic_model )
		return false;

	planning_scene_ = new planning_scene::PlanningScene(kinematic_model);

	if ( not planning_scene_ )
		return false;

	if ( not updatePlanningScene() )
		return false;

	ROS_INFO("MoveIt! configuration loaded");

	move_group_ = new moveit::planning_interface::MoveGroup(arm_prefix_);

	addWall();
	return true;
}

bool ArmControllerKinova::updatePlanningScene()
{	
	moveit_msgs::GetPlanningScene srv;
	srv.request.components.components = 
		srv.request.components.SCENE_SETTINGS |
		srv.request.components.ROBOT_STATE |
		srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS |
		srv.request.components.WORLD_OBJECT_NAMES |
		srv.request.components.WORLD_OBJECT_GEOMETRY |
		srv.request.components.OCTOMAP |
		srv.request.components.TRANSFORMS |
		srv.request.components.ALLOWED_COLLISION_MATRIX |
		srv.request.components.LINK_PADDING_AND_SCALING |
		srv.request.components.OBJECT_COLORS;

	// Make sure client is connected to server
	if ( not planning_scene_service_client_.exists() )
	{
	  ROS_ERROR_ONCE("Service %s does not exist.", planning_scene_service_client_.getService().c_str());
	  return false;
	}

	ROS_DEBUG("Calling planning scene service");
	if ( planning_scene_service_client_.call(srv) )
   		planning_scene_->usePlanningSceneMsg(srv.response.scene);
	else
	{
   		ROS_WARN_ONCE("Failed to call service %s", planning_scene_service_client_.getService().c_str());
   		return false;
	}

	return true;
}

bool ArmControllerKinova::addWall()
{
	ROS_INFO("Adding box");

	planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames(false));

	moveit_msgs::CollisionObject 	collision_object;
	shape_msgs::SolidPrimitive 		primitive;

	collision_object.id 				= "box1";

	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3); 
	primitive.dimensions[0] = 5.0;
	primitive.dimensions[1] = 0.05;
	primitive.dimensions[2] = 5.0;

	geometry_msgs::Pose box_pose; //,box_pose1,box_pose2,box_pose3;
	box_pose.orientation.w 	= 1.0;
	box_pose.position.x 	= 0.0;
	box_pose.position.y 	= 0.15;
	box_pose.position.z 	= 0.0;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;  
	collision_objects.push_back(collision_object);  

	ROS_INFO("Add objects into the world");  
	planning_scene_interface_.addCollisionObjects(collision_objects);

	return true;
}

void ArmControllerKinova::CB_joint_state_received(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	std::lock_guard<std::mutex> lock(joint_states_mutex_);

	joint_states_initialized_ = true;
	joint_states_ 			  = *joint_state;
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

bool ArmControllerKinova::inCollision()
{
	std::lock_guard<std::mutex> lock(colision_mutex_);

	return in_collision_;
}

bool ArmControllerKinova::updateCollisions()
{	
	if (planning_scene_ == NULL)
	{
		ROS_ERROR("No planning scene set");
		return false; 
	}

	if ( not updatePlanningScene() )
		return false;

	collision_detection::CollisionRequest 	collision_request;
	collision_detection::CollisionResult 	collision_result;

	collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();  
	robot_state::RobotState copied_state 			= planning_scene_->getCurrentState();  

	collision_result.clear();
	planning_scene_->checkCollision(collision_request, collision_result, copied_state, acm);

	colision_mutex_.lock();
	in_collision_ = collision_result.collision;
	colision_mutex_.unlock();

	ROS_DEBUG_NAMED("Test: Current state is %s collision", (collision_result.collision ? "in" : "not in"));  
	
	if ( collision_result.collision )
		ROS_WARN("Collision detected!");

	return true;
}

bool ArmControllerKinova::showEndEffectorGoalPose( const geometry_msgs::Pose& pose )
{
	visualization_msgs::Marker marker;
    marker.header.frame_id 	= name_;
    marker.header.stamp 	= ros::Time();
    marker.ns 				= arm_prefix_;
    marker.id 				= 123;
    marker.type 			= visualization_msgs::Marker::ARROW;
    marker.action 			= visualization_msgs::Marker::ADD;
    marker.pose 			= pose;
	marker.scale.x 			= 0.1;
	marker.scale.y 			= 0.1;
	marker.scale.z 			= 0.1;
	marker.color.a 			= 1.0; // Don't forget to set the alpha!
	marker.color.r 			= 1.0;
	marker.color.g 			= 0.0;
	marker.color.b 			= 0.0;
	marker.lifetime 		= ros::Duration(); // A value of ros::Duration() means never to auto-delete

   visualization_pub_.publish(marker);

   return true;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerKinova, arm_controller_base::ArmControllerBase);
