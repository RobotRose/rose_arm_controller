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
#include "arm_controller_robai/arm_controller_robai.hpp"

namespace arm_controller_plugins {

ArmControllerRobai::ArmControllerRobai()
	: n_("~robai_arm")
	, end_effector_mode_(POINT_EE)
	, control_mode_(POSITION)
	, manipulation_action_manager_()
	, emergency_(false)
{
	// Load the parameters that are needed
	loadParameters();
}

ArmControllerRobai::~ArmControllerRobai()
{

}

bool ArmControllerRobai::initialize()
{
	ROS_INFO("Initializing Robai arm...");
	return connectToArms();
}

bool ArmControllerRobai::close()
{
	shutdown();
	return true;
}

bool ArmControllerRobai::cancel()
{
	ROS_INFO("Cancel received");
	return manipulation_action_manager_.cancelManipulation();
}

bool ArmControllerRobai::emergencyStop()
{
	emergency_ = true;

	return cancel();
}

bool ArmControllerRobai::resetEmergencyStop()
{
	emergency_ = false;

	return true;
}

int ArmControllerRobai::getNumberOfJoints() 
{
	return getNumJoints();
}

bool ArmControllerRobai::getEndEffectorPose(Pose& pose)
{
	//! @todo MdL: Check function during runtime.
	EcManipulatorEndEffectorPlacement actual_arms_placement;
	EcCoordinateSystemTransformation  actual_coordinate;
	
	if ( not getActualPlacement(actual_arms_placement))
	{	
		ROS_ERROR("Could not get end effector pose");
		return false;
	}

	actual_coordinate  = actual_arms_placement.offsetTransformations()[getRobaiArmIndex()].coordSysXForm();

    pose.position.x    = actual_coordinate.translation().x();
    pose.position.y    = actual_coordinate.translation().y();
    pose.position.z    = actual_coordinate.translation().z();
    pose.orientation.x = actual_coordinate.orientation().x();
    pose.orientation.y = actual_coordinate.orientation().y();
    pose.orientation.z = actual_coordinate.orientation().z();
    pose.orientation.w = actual_coordinate.orientation().w();

	return true;
}

bool ArmControllerRobai::setEndEffectorPose(const Pose& end_effector_pose)
{	
	if (emergency_)
		return false;

	ROS_INFO("ArmControllerRobai::setEndEffectorPose: (%f,%f,%f)",
        end_effector_pose.position.x, 
        end_effector_pose.position.y, 
        end_effector_pose.position.z);

	// Pose correction from gripper to the wrist joint (the wrist joint is controlled by the Robai path planning software)
	geometry_msgs::Pose corrected_pose = getCorrectedEndEffectorPose(end_effector_pose);

	ROS_INFO("ArmControllerRobai::corrected pose: (%f,%f,%f)",
        corrected_pose.position.x, 
        corrected_pose.position.y, 
        corrected_pose.position.z);

	return manipulation_action_manager_.executePoseManipulation(getRobaiArmIndex(), getRobaiEndEffectorMode(), corrected_pose);
}

bool ArmControllerRobai::getEndEffectorVelocity(Twist& twist)
{
	//! @todo MdL: Implement.
	return false;
}

bool ArmControllerRobai::setEndEffectorVelocity(const Twist& velocity)
{
	if (emergency_)
		return false;

	ROS_INFO("ArmControllerRobai::setVelocity: (%f,%f,%f):(%f,%f,%f)",
                    velocity.linear.x, velocity.linear.y, velocity.linear.z, velocity.angular.x,
                    velocity.angular.y, velocity.angular.z);

    if ( not setVelocityControl())
    	return false;

    if (getEndEffectorMode() != FRAME_EE) 
    	if ( not setEndEffectorMode(FRAME_EE))
    		return false;

    std::vector<double> velocity_vector;
    velocity_vector.push_back(velocity.linear.x);
    velocity_vector.push_back(velocity.linear.y);
    velocity_vector.push_back(velocity.linear.z);

    velocity_vector.push_back(velocity.angular.x);
    velocity_vector.push_back(velocity.angular.y);
    velocity_vector.push_back(velocity.angular.z);

    if (not setDesiredVelocity(velocity_vector, ARM, getRobaiArmIndex()))
    {
        ROS_ERROR("Could not set desired velocity");
        return false;
    }

	return true;
}

bool ArmControllerRobai::getConstraints(Twist& twist)
{
	if (getEndEffectorMode() == FRAME_EE)
	{
		//! @todo MdL: Verify.
		twist.angular.x = 1.0;
		twist.angular.y = 1.0;
		twist.angular.z = 1.0;
	}
	else if (getEndEffectorMode() == FREE_SPIN)
	{
		//! @todo MdL: Verify.
		twist.angular.x = 1.0;
		twist.angular.y = 1.0;
	}
	else if (getEndEffectorMode() == POINT_EE)
	{
		// Do nothing (all zero values)
	}
	else
	{
		ROS_ERROR("Could not get constraints");
		return false;
	}
	
	return true;
}

bool ArmControllerRobai::setConstraints(const Twist& constraint)
{
	if(emergency_)
		return false;

	ROS_INFO("setConstraints request received");

	if (constraint.linear.x > 0)
		ROS_WARN("The robai arm does not allow for setting linear constraints");
	if (constraint.linear.y > 0)
		ROS_WARN("The robai arm does not allow for setting linear constraints");
	if (constraint.linear.z > 0)
		ROS_WARN("The robai arm does not allow for setting linear constraints");
	if (constraint.angular.x > 0)
		if (constraint.angular.y > 0)
			if (constraint.angular.z > 0)
				return setEndEffectorMode(FRAME_EE);
			else
				return setEndEffectorMode(FREE_SPIN); //! @todo MdL: Verify.
		else // angular.y == 0
			ROS_WARN("No mode possible for angular constraint: (%f, %f, %f)", constraint.angular.x, constraint.angular.y, constraint.angular.z);
	else
		if (constraint.angular.y > 0)
			ROS_WARN("No mode possible for angular constraint: (%f, %f, %f)", constraint.angular.x, constraint.angular.y, constraint.angular.z);	
		if (constraint.angular.z > 0)
			ROS_WARN("No mode possible for angular constraint: (%f, %f, %f)", constraint.angular.x, constraint.angular.y, constraint.angular.z);
		return setEndEffectorMode(POINT_EE);

	return false;
}

bool ArmControllerRobai::resetConstraints()
{
	return setEndEffectorMode(POINT_EE);
}

double ArmControllerRobai::getGripperWidth()
{
	//! @todo MdL: Implement.
	return 0.0;
}

bool ArmControllerRobai::setGripperWidth(const double required_width)
{	
	if(emergency_)
		return false;

	//! @todo MdL: Map width to percentage min/max width min_gripper width and max gripper width.
	// For now: Smaller than 0.01 is closed, more than 0.0 is open
	int percentage_open = (required_width < 0.01 ? 0: 100);
	return manipulation_action_manager_.executeGripperManipulation(getRobaiGripperIndex(), getRobaiEndEffectorMode(), percentage_open);
}

bool ArmControllerRobai::getEndEffectorWrench(Wrench& wrench)
{
	ROS_ERROR("Robai does not support reading force/torque values");
	return false;
}

bool ArmControllerRobai::setEndEffectorWrench(const Wrench& Wrench)
{
	if(emergency_)
		return false;

	ROS_ERROR("Robai does not support force/torque control");
	return false;
}

bool ArmControllerRobai::getJointPositions(vector<double>& joint_positions)
{
    if (not getJointValues(joint_positions)) 
    {
    	ROS_ERROR("Could not get joint values");
		return false;
    }
    
	return false;
}

bool ArmControllerRobai::getJointVelocities(vector<double>& joint_velocities)
{
	ROS_ERROR("Cannot read joint velocities");
	return false;
}

bool ArmControllerRobai::getJointEfforts(vector<double>& joint_angular_forces)
{
	ROS_ERROR("Cannot read joint efforts");
	return false;
}

bool ArmControllerRobai::connectToArms(const std::string ip)
{
    ROS_INFO("Connecting to arms...");

    ros::Rate r(1);

    bool connected = false;
    while (not connected)
    {
        try
        {
            if (init(ip))
            {
                connected = true;
                ROS_DEBUG("Connected to ip %s", ip.c_str());
            }
        }
        catch (const boost::thread_interrupted& ex)
        {
            ROS_ERROR("Received a boost::thread_interrupted exception");

            ROS_DEBUG("Could not connect to ip %s", ip.c_str());
            ROS_DEBUG("Retrying");
        }

        r.sleep();
    }
    ROS_INFO("Connected!");
    return true;
}

bool ArmControllerRobai::loadParameters()
{
    ROS_INFO("Loading robai arm parameters...");

    //! @todo MdL: Check is parameters are loaded via a file.
    // if(not )
    //    ROS_WARN("Gripper tip correction was not set in confugation file, defaulting to %f", gripper_tip_correction_parameter_);

    n_.param("/robai_configuration/gripper_tip_correction", gripper_tip_correction_parameter_, 0.155);

    ROS_INFO("Done.");

    return true;
}

bool ArmControllerRobai::setEndEffectorMode ( const ArmControllerRobai::EndEffectorMode& end_effector_mode, const bool save_state )
{
    ROS_DEBUG("Setting new end effector mode %d", (int)end_effector_mode);

	if (end_effector_mode_ == end_effector_mode) 
    	return true;
    
    if ( not setEndEffectorSet(end_effector_mode, getRobaiArmIndex()))
    {
     	ROS_ERROR("Could not set end effector mode");
        return false;
    }
    
    if (save_state)
    	end_effector_mode_ = end_effector_mode;

    return true;
}

ArmControllerRobai::EndEffectorMode ArmControllerRobai::getEndEffectorMode()
{
    return end_effector_mode_;
}

int ArmControllerRobai::getRobaiEndEffectorMode()
{
    return static_cast<int>(end_effector_mode_);
}

bool ArmControllerRobai::resetEndEffectorSet()
{
	if ( not setEndEffectorSet(end_effector_mode_, getRobaiArmIndex()))
    {
     	ROS_ERROR("Could not set end effector mode");
        return false;
    }

    return true;
}

bool ArmControllerRobai::disallowArmMovement()
{
    return setEndEffectorMode(NO_MOVEMENT_ARM_ALLOWED, false);
}

Pose ArmControllerRobai::getCorrectedEndEffectorPose(const Pose& pose)
{
	ROS_DEBUG("Correcting pose (%f,%f,%f):(%f,%f,%f,%f) with length %f", 
		pose.position.x, 
		pose.position.y, 
		pose.position.z,
		pose.orientation.x,
		pose.orientation.y,
		pose.orientation.z,
		pose.orientation.w,
		gripper_tip_correction_parameter_
	);
	//! @todo MdL: Test has to be really tested...

	tf::Quaternion 		base_quaternion(0.0, 0.0, 0.0, 1.0);
	tf::Vector3 		base_vector(0.0, 0.0, -gripper_tip_correction_parameter_);
	tf::Transform 		base(base_quaternion, base_vector); 

	tf::Quaternion 		end_effector_quaternion(0.0, 0.0, 0.0, 1.0);
	tf::quaternionMsgToTF(pose.orientation, end_effector_quaternion);

	tf::Vector3 		end_effector_vector(pose.position.x, pose.position.y, pose.position.z);
	tf::Transform 		end_effector_pose(end_effector_quaternion, end_effector_vector); 

	// geometry_msgs::Quaternion temp_quat;
	// temp_quat.w = 1.0;
	// tf::quaternionTFToMsg(base.getRotation(), temp_quat);
	ROS_DEBUG("Base (%f,%f,%f):(%f,%f,%f,%f)", 
		base.getOrigin().getX(), 
		base.getOrigin().getY(), 
		base.getOrigin().getZ(),
		base.getRotation().getAxis().getX(),
		base.getRotation().getAxis().getY(),
		base.getRotation().getAxis().getZ(),
		base.getRotation().getW()
	);	

	// tf::quaternionTFToMsg(end_effector_pose.getRotation(), temp_quat);
	ROS_DEBUG("End effector required pose(%f,%f,%f):(%f,%f,%f,%f)", 
		end_effector_pose.getOrigin().getX(), 
		end_effector_pose.getOrigin().getY(), 
		end_effector_pose.getOrigin().getZ(),
		end_effector_pose.getRotation().getAxis().getX(),
		end_effector_pose.getRotation().getAxis().getY(),
		end_effector_pose.getRotation().getAxis().getZ(),
		end_effector_pose.getRotation().getW()
	);

	tf::Transform result_transform = base*end_effector_pose;

	ROS_DEBUG("End effector pose converted to (%f,%f,%f):(%f,%f,%f,%f)", 
		result_transform.getOrigin().getX(), 
		result_transform.getOrigin().getY(), 
		result_transform.getOrigin().getZ(),
		result_transform.getRotation().getAxis().getX(),
		result_transform.getRotation().getAxis().getY(),
		result_transform.getRotation().getAxis().getZ(),
		result_transform.getRotation().getW()
	);

    //! @todo MdL: Fix this hack for singlehand.
    // TFHelper* goal_tf = new TFHelper("end_effector_goal", n_, "left_arm", "/end_effector_goal");

	Pose result;
	result.position.x = result_transform.getOrigin().getX();
	result.position.y = result_transform.getOrigin().getY();
	result.position.z = result_transform.getOrigin().getZ();

	result.orientation.x = result_transform.getRotation().getAxis().getX();
	result.orientation.y = result_transform.getRotation().getAxis().getY();
	result.orientation.z = result_transform.getRotation().getAxis().getZ();
	result.orientation.w = result_transform.getRotation().getW();

	ROS_DEBUG("Result (%f,%f,%f):(%f,%f,%f,%f)", 
		result.position.x, 
		result.position.y, 
		result.position.z,
		result.orientation.x,
		result.orientation.y,
		result.orientation.z,
		result.orientation.w
	);

    return result;
}

int ArmControllerRobai::getRobaiArmIndex()
{
    return 0;
}

int ArmControllerRobai::getRobaiGripperIndex()
{
    return 1;
}

bool ArmControllerRobai::setPositionControl()
{
    if (control_mode_ == POSITION) 
    	return true;

    // Control mode to set (0 for position control and 1 for velocity control)
    if (not setControlMode(0, getRobaiArmIndex()))
    {
        ROS_ERROR("Could not set control mode to velocity control");
        return false;
    }

    control_mode_ = POSITION;
    return true;
}

bool ArmControllerRobai::setVelocityControl()
{
    if (control_mode_ == VELOCITY) 
    	return true;

    // Control mode to set (0 for position control and 1 for velocity control)
    if (not setControlMode(1, getRobaiArmIndex()))
    {
        ROS_ERROR("Could not set control mode to velocity control");
        return false;
    }

    control_mode_ = VELOCITY;
    return true;
}

}; // namespace

PLUGINLIB_EXPORT_CLASS(arm_controller_plugins::ArmControllerRobai, arm_controller_base::ArmControllerBase);
