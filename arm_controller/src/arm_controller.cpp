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
#include "arm_controller/arm_controller.hpp"

ArmController::ArmController( std::string name, ros::NodeHandle n )
    : name_ ( name )
    , n_ ( n )
    , sh_emergency_(SharedVariable<bool>("emergency"))
    , velocity_watchdog_("arm_velocity_watchdog", n, VELOCITY_TIMEOUT, boost::bind(&ArmController::cancelVelocityForArms, this))
{
    ROS_INFO_NAMED(ROS_NAME, "Starting arm controller...");

    // Load all parameters
    int                         number_of_arms;
    std::vector<std::string>    arm_plugins;
    
    n_.param("/arm_controller/number_of_arms", number_of_arms, 1);
    for ( int i = 0 ; i < number_of_arms ; i++ )
    {
        std::string plugin_name;
        n_.param("/arm_controller/plugin"+i, plugin_name, "arm_controller_plugins::ArmControllerRobai");
        arm_plugins.push_back(plugin_name);
    }
    
    // Load plugins
    std::vector<pluginlib::ClassLoader> arm_controllers;
    for ( int i = 0 ; i < number_of_arms ; i++)
    {
        pluginlib::ClassLoader<arm_plugins[i]> arm_controller("arm_controller_plugins", "arm_controller_base::ArmControllerBase");

    	try
    	{
    		boost::shared_ptrarm_plugins[i]> arm = arm_controller.createInstance(arm_plugins[i]);
    	}
    		catch(pluginlib::PluginlibException& ex)
    	{
    		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    	}

        arm_controllers.push_back(arm_controller);
    }

    // Create SMC
    smc_ = new SMC(n_, name_, boost::bind(&ArmController::CB_receiveGoal, this, _1, _2),
                              boost::bind(&ArmController::CB_serverCancel, this, _1));
    // Init member variables
    obstacle_map_ = map<Manipulators, Pose>();

    attached_items = map<Arms, std::string>();

    for ( int arm = 0 ; arm < static_cast<int>(ArmController::Arms::total) ; arm++ )
    {
        setPrerecordedVisualCorrection(static_cast<ArmController::Arms>(arm), false);
        setVisualCorrection(static_cast<ArmController::Arms>(arm), false);
        resetRecordedVisualCorrection(static_cast<ArmController::Arms>(arm));
    }

    // Enable services / clients / publishers
    toggle_service_         = n_.advertiseService("/" + name_ + "/toggle_visual_correction", &ArmController::CB_toggle, this);
    reset_service_          = n_.advertiseService("/" + name_ + "/reset_visual_correction",  &ArmController::CB_reset,  this);
    attach_item_service_    = n_.advertiseService("/" + name_ + "/set_item_attachment",      &ArmController::CB_attach_item,  this);
    query_attached_items_service_    = n_.advertiseService("/" + name_ + "/get_item_attachment",      &ArmController::CB_query_attached_items,  this);

    gaze_client_        = new GazeClient("gaze_controller", true);

    vis_pub_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    // Monitor the emergency button state
    sh_emergency_.connect(ros::Duration(0.1));
    sh_emergency_.registerChangeCallback(boost::bind(&ArmController::CB_emergencyCancel, this,  _1));

    ROS_INFO_NAMED(ROS_NAME, "Arm controller ready");
}

ArmController::~ArmController()
{
}

bool ArmController::checkInputs( const Arms arm )
{
    //! @todo MdL: Allow for more checks.
    if ( static_cast<int>(arm) < 0 || static_cast<int>(arm) >= total )
    {
        ROS_ERROR( "No arm selected, arm id: %d", static_cast<int>(arm));
        return false;
    }

    return true;
}

void ArmController::CB_receiveGoal( const arm_controller::manipulateGoalConstPtr& goal, SMC* smc )
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_receiveGoal");
    map<Manipulators, Pose> obstacle_map;

    if(sh_emergency_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Will not move arm due to emergency state.");
        sendResult(ACTION_RESULT::NO_ARM_MOVEMENT_DUE_TO_EMERGENCY);
        return;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::Choose required actions");
    switch( goal->required_action )
    {
        case MOVE_TO_POSE:
            if (checkInputs(static_cast<Arms>(goal->arm)))
                moveToPoseService( static_cast<Arms>(goal->arm), goal->required_pose, static_cast<EndEffectorMode>(goal->required_end_effector_mode), obstacle_map );
            else
                sendResult(ACTION_RESULT::WRONG_INPUT_ERROR);
            break;

        case MOVE_GRIPPER:
            if (checkInputs(static_cast<Arms>(goal->arm)))
                moveGripperService( static_cast<Arms>(goal->arm), goal->required_gripper_width, obstacle_map );
            else
                sendResult(ACTION_RESULT::WRONG_INPUT_ERROR);
            break;

        case SET_JOINT_POSITIONS:
            if (checkInputs(static_cast<Arms>(goal->arm)))
                setJointPositionsService( static_cast<Arms>(goal->arm), goal->required_joint_positions, obstacle_map );
            else
                sendResult(ACTION_RESULT::WRONG_INPUT_ERROR);
            break;

        case SET_VELOCITY:
            if (checkInputs(static_cast<Arms>(goal->arm)))
                setVelocityService( static_cast<Arms>(goal->arm), goal->required_velocity, obstacle_map );
            else
                sendResult(ACTION_RESULT::WRONG_INPUT_ERROR);
            break;

        case MOVE_RELATIVE_POSE:
            if (checkInputs(static_cast<Arms>(goal->arm)))
                moveToRelativePoseService( static_cast<Arms>(goal->arm), goal->required_pose, static_cast<EndEffectorMode>(goal->required_end_effector_mode), obstacle_map );
            else
                sendResult(ACTION_RESULT::WRONG_INPUT_ERROR);
            break;

        case SET_OBSTACLES:
            if ( goal->obstacles.size() != goal->obstacle_poses.size() )
            {
                ROS_ERROR("Number of obstacles is not equal to the number of poses given. #obstacles: %d, #poses: %d", (int)goal->obstacles.size(), (int)goal->obstacle_poses.size());
                sendResult(ACTION_RESULT::WRONG_INPUT_ERROR);
            }
            else
            {
                for ( int i = 0 ; i < goal->obstacles.size(); i++ )
                    obstacle_map.insert(std::pair<Manipulators, Pose>(static_cast<Manipulators>(goal->obstacles[i]),goal->obstacle_poses[i]));

                setObstaclesService( obstacle_map );
            }
            break;

        case REMOVE_OBSTACLES:
            removeObstaclesService( goal->obstacles );
            break;

        case CLEAR_OBSTACLES:
            clearObstaclesService();
            break;

        default:
            actionNotDefinedService();
    }
}

void ArmController::CB_serverCancel( SMC* smc )
{
    cancel();
}

void ArmController::CB_emergencyCancel(const bool& new_value)
{
    ROS_INFO_NAMED(ROS_NAME, "arm CB_emergencyCancel"); //! @todo OH: REMOVE!

    if( new_value == true )
        cancel();
}

void ArmController::removeObstaclesService( const vector<int> goal_manipulators )
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::removeObstaclesService");

    ACTION_RESULT result_message;
    for ( const auto& manipulator : goal_manipulators )
    {
        result_message = removeFromScene(static_cast<Manipulators>(manipulator));
        if ( result_message != ACTION_RESULT::SUCCESS )
        {
            sendResult(ACTION_RESULT::UNKNOWN_ERROR);
            return;
        }

        obstacle_map_.erase(static_cast<Manipulators>(manipulator));
    }

    sendResult(result_message);
}

void ArmController::clearObstaclesService()
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::clearObstaclesService");

    ACTION_RESULT result_message;
    for ( int obstacle = static_cast<int>(Manipulators::TABLE) ; obstacle < static_cast<int>(Manipulators::last) ; obstacle++ )
    {
        result_message = removeFromScene(static_cast<Manipulators>(obstacle));
        if ( result_message != ACTION_RESULT::SUCCESS )
            break;
    }

    sendResult(result_message);

    ROS_DEBUG_NAMED(ROS_NAME, "Obstacles cleared");

    obstacle_map_.clear();
}

void ArmController::setObstaclesService( const map<Manipulators, Pose> obstacle_map )
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::setObstaclesService");
    sendResult(addObstacles(obstacle_map));
}

void ArmController::cancelVelocityForArms()
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    setVelocityService(Arms::RIGHT, twist);
    setVelocityService(Arms::LEFT, twist);

    velocity_watchdog_.stop();
}

void ArmController::setVelocityService( const Arms arm, const geometry_msgs::Twist twist, const map<Manipulators, Pose> obstacle_map )
{
    velocity_watchdog_.reset();

    ACTION_RESULT result_message;

    result_message = setVelocity(arm, twist);
    sendResult(result_message);
}

ACTION_RESULT ArmController::addObstacles( const map<Manipulators, Pose> obstacle_map )
{
    // Compare old and new obstacle maps, if the obstacles were removed: Move them 'away' in the environment
    vector<Manipulators> current_obstacles = getKeys(obstacle_map_);
    vector<Manipulators> new_obstacles     = getKeys(obstacle_map);

    // for ( auto& current_obstacle : current_obstacles )
    //     if ( find(new_obstacles.begin(), new_obstacles.end(), current_obstacle) == new_obstacles.end() )
    //         removeFromScene(current_obstacle);

    for(auto it = obstacle_map.begin() ; it != obstacle_map.end() ; it++) 
        if ( setObstacle(it->first, it->second) != ACTION_RESULT::SUCCESS )
            return ACTION_RESULT::UNKNOWN_ERROR;

    obstacle_map_ = obstacle_map;

    ROS_DEBUG_NAMED(ROS_NAME, "The map of obstacle now contains %d obstacles", (int)obstacle_map_.size());

    return ACTION_RESULT::SUCCESS;
}

ACTION_RESULT ArmController::removeFromScene( const Manipulators obstacle )
{
    ROS_DEBUG_NAMED(ROS_NAME, "Removing obstacle with ID %d from the scene", (int)obstacle);
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = -10.0;
    pose.position.z = 10.0;
    pose.orientation.w = 1.0;

    return setObstacle(obstacle, pose);
}

vector<ArmController::Manipulators> ArmController::getKeys( const map<Manipulators, Pose> map )
{
    vector<Manipulators> keys;
       
    for( auto it = map.begin(); it != map.end(); ++it ) 
        keys.push_back(it->first);

    return keys;
}

void ArmController::moveGripperService( const Arms arm, const double width, const map<Manipulators, Pose> obstacle_map )
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::moveGripperServices");
    ACTION_RESULT result_message;

    if ( width < 0 || width > 100 )
    {
        ROS_ERROR( "moveGripperService: Gripper width should be between 0 and 1");
    }

    // result_message = addObstacles(obstacle_map);
    // if ( result_message != ACTION_RESULT::SUCCESS )
    // {
    //     sendResult(result_message);
    //     return;
    // }

    double scaled_width = std::max(5,(int)width); //( width*(get_gripper_max_width() - get_gripper_min_width()) + get_gripper_min_width() );
    scaled_width = std::min(95,(int)scaled_width);
    
    result_message = moveGripper(arm, scaled_width);
    sendResult(result_message);
}

void ArmController::moveToRelativePoseService( const Arms arm, Pose pose, const EndEffectorMode mode, const map<Manipulators, Pose> obstacle_map )
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::moveToRelativePoseService");

    // if ( not resultSuccessful(addObstacles(obstacle_map)) )
    //     return;

    ROS_DEBUG_NAMED(ROS_NAME, "Add x: %f, y: %f, z: %f", pose.position.x, pose.position.y, pose.position.z);

    sendResult(moveToRelativePose(arm, pose, mode));
    return;
}

void ArmController::moveToPoseService( const Arms arm, Pose pose, const EndEffectorMode mode, const map<Manipulators, Pose> obstacle_map )
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::moveToPoseService");
    publishTargetMarker(arm, pose);

    // if ( calibrating_function_ != NULL )
    //      calibrating_function_(&pose);

    // if ( not resultSuccessful(addObstacles(obstacle_map)) )
    //     return;

    // Apply a prerecorded correction to prevent an error 
    if( prerecordedVisualCorrectionSet(arm) )
        applyPrerecordedVisualCorrection(arm, pose);

    if( visualCorrectionSet(arm) )
    {
        if ( not resultSuccessful(moveToPose(arm, pose, mode)) )
        {
            ROS_DEBUG_NAMED(ROS_NAME, "ArmController::moveToPoseService: moveToPose not successfull");
            return;
        }
        
        ROS_DEBUG_NAMED(ROS_NAME, "ArmController::moveToPoseService: Doing visual correction for %s arm", arm == 0 ? "right" : "left");
        
        lookAtHandAndTrack(arm);

        applyVisualCorrection(arm, pose);

        // Move to the corrected pose
        sendResult(moveToPose(arm, pose, mode));
        
        // Stop tracking
        lookAtHand(arm, false);

        return;
    }
    else
    {
        sendResult(moveToPose(arm, pose, mode));
        return;
    }
    
    sendResult(ACTION_RESULT::SUCCESS);
    return;
}

void ArmController::applyPrerecordedVisualCorrection( const Arms arm, Pose& pose ) 
{
    ROS_INFO_NAMED(ROS_NAME, "Applying a prerecorded error correction as preventive correction");

    Pose prerecorded_error = getRecordedVisualCorrection(arm);
    correctPoseWithError(prerecorded_error, pose);
}

void ArmController::applyVisualCorrection( const Arms arm, Pose& pose ) 
{
    ROS_INFO_NAMED(ROS_NAME, "Trying to apply visual correction");

    if ( recordVisualCorrection(arm) )
    {
        correctPoseWithError(getRecordedVisualCorrection(arm), pose);
        ROS_INFO_NAMED(ROS_NAME, "ArmControllerRobaiBihand::moveToPose: Corrected pose has x=%f, y=%f, z=%f", 
            pose.position.x, 
            pose.position.y, 
            pose.position.z);
    }
    else
        ROS_ERROR("Could not apply visual correction");
}

bool ArmController::resultSuccessful( const ACTION_RESULT result_message )
{
    if ( result_message != ACTION_RESULT::SUCCESS )
    {
        sendResult(result_message);
        return false;
    }
    
    return true;
}

void ArmController::setJointPositionsService( const Arms arm, const vector<double> joint_positions, const map<Manipulators, Pose> obstacle_map )
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::setJointPositionsService");
    ACTION_RESULT result_message;

    // result_message = addObstacles(obstacle_map);
    // if ( result_message != ACTION_RESULT::SUCCESS )
    // {
    //     sendResult(result_message);
    //     return;
    // }

    result_message = setJointPositions(arm, joint_positions, 0.01);
    sendResult(result_message);
}

void ArmController::actionNotDefinedService()
{
    sendFail(ACTION_RESULT::ARM_MANIPULATION_NOT_DEFINED_ERROR);
}

void ArmController::sendResult( const ACTION_RESULT& message ) 
{
    if ( not smc_->hasActiveGoal() )
        return;

    if ( message == ACTION_RESULT::SUCCESS )
        sendSuccess();
    else   
        sendFail(message);
}   

void ArmController::sendSuccess()
{
    arm_controller::manipulateResult result;
    result.return_code = ACTION_RESULT::SUCCESS;
    // result.end_pose = getEndEffectorPose(static_cast<Arms>((smc_->getLastGoal())->arm));

    smc_->sendServerResult<arm_controller::manipulateAction>( true, result );
}

void ArmController::sendFail( const ACTION_RESULT action_result )
{
    arm_controller::manipulateResult result;
    result.return_code = action_result;
    // result.end_pose = getEndEffectorPose(static_cast<Arms>((smc_->getLastGoal())->arm));

    smc_->sendServerResult<arm_controller::manipulateAction>( false, result );
}

bool ArmController::calculateError(const std::string& expected_frame, const std::string& measured_frame, Pose& error_pose)
{       
    try
    {
        tf::StampedTransform error_transform;
        // If we can see the AR-marker on the arm, we can use the observated position to estimate the difference between the observed pose and the expected pose
        // The expected pose is simply what we send/measure to/from the arm driver.
        tf_.waitForTransform(expected_frame, measured_frame, ros::Time(0), ros::Duration(MAX_WAITING_TIME)); //Wait MAX_WAITING_TIME, otherwise it takes too long
        tf_.lookupTransform(expected_frame, measured_frame, ros::Time(0), error_transform);
        tf::Vector3    position    = error_transform.getOrigin();
        tf::Quaternion orientation = error_transform.getRotation();

        error_pose.position.x  = position.x();
        error_pose.position.y  = position.y();
        error_pose.position.z  = position.z();
        tf::quaternionTFToMsg(orientation, error_pose.orientation);

        ROS_DEBUG_NAMED(ROS_NAME, "ArmController::calculateError: Error: dX: %f, dY: %f, dZ: %f in frame %s", error_pose.position.x, 
                                                                   error_pose.position.y, 
                                                                   error_pose.position.z, expected_frame.c_str());
        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Probably could not find AR-marker for correction, received an exception : %s", ex.what());
        return false;
    }
}

bool ArmController::correctPoseWithError(const Pose& error_pose, Pose& pose_to_correct)
{
    ROS_INFO_NAMED(ROS_NAME, "ArmController::correctPoseWithError: uncorrected pose has x=%f, y=%f, z=%f", 
            pose_to_correct.position.x, pose_to_correct.position.y, pose_to_correct.position.z);

    Pose zero_pose;
    double error_distance = rose20_common::getVectorLengthXYZ(error_pose.position.x, error_pose.position.y, error_pose.position.z);

    if(error_distance <= MAX_ERROR)
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Correction vector is %f (< %f), applying correction", error_distance, MAX_ERROR);
        pose_to_correct.position.x -= error_pose.position.x;
        pose_to_correct.position.y -= error_pose.position.y;
        pose_to_correct.position.z -= error_pose.position.z;

        //! @todo MdL: Add orientation correction too.
        // pose_to_correct.orientation.x += error_pose.orientation.x;
        // pose_to_correct.orientation.y += error_pose.orientation.y;
        // pose_to_correct.orientation.z += error_pose.orientation.z;
        // pose_to_correct.orientation.w += error_pose.orientation.w;

        ROS_DEBUG_NAMED(ROS_NAME, "corrected pose: Position: X: %f, Y: %f, Z: %f. Orientation: x: %f, y: %f, z: %f, w: %f", 
            pose_to_correct.position.x, 
            pose_to_correct.position.y, 
            pose_to_correct.position.z,
            pose_to_correct.orientation.x,
            pose_to_correct.orientation.y,
            pose_to_correct.orientation.z,
            pose_to_correct.orientation.w);

        return true;
    }
    else
        ROS_WARN_NAMED(ROS_NAME, "Not applying correction, it is larger than %f (%f) so unsafe", MAX_ERROR, error_distance);

    return false;
}

bool ArmController::recordVisualCorrection(const Arms arm)
{   
    Pose current_error;
    if (not calculateError(toString(arm)+"_grippermarker_expected", toString(arm)+"_grippermarker_observed", current_error))
    {
        ROS_ERROR("Could not calculate error");
        return false;
    }

    Pose previous_error = getRecordedVisualCorrection(arm);

    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::recordVisualCorrection: Previous: X: %f, Y: %f, Z: %f. ", 
                                                                 previous_error.position.x, 
                                                                 previous_error.position.y, 
                                                                 previous_error.position.z);

    Pose accumulated_error;

    accumulated_error.position.x = previous_error.position.x + current_error.position.x;
    accumulated_error.position.y = previous_error.position.y + current_error.position.y;
    accumulated_error.position.z = previous_error.position.z + current_error.position.z;

    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::recordVisualCorrection: Accumulated = previous + current error: X: %f, Y: %f, Z: %f. ", 
                                                                accumulated_error.position.x, 
                                                                accumulated_error.position.y, 
                                                                accumulated_error.position.z);
    
    setRecordedVisualCorrection(arm, accumulated_error);

    publishCorrectionVector(arm, accumulated_error);

    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::recordVisualCorrection: Accumulated error stored");

    return true;
}

const bool ArmController::lookAtHandAndTrack( const Arms arm )
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::lookAtHandAndTrack");
    // Look at known position of hand, but do not track
    inspectHand(arm);

    // Track expected marker on hand
    lookAtHand(arm, true); 
}

const bool ArmController::inspectHand( const Arms arm )
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::inspectHand");
    // Moving image is better for finding the marker
    if ( not lookAt("/"+toString(arm)+"_gripper", false)) 
        return false;
    if ( not lookAt("/"+toString(arm)+"_wrist", false)) 
        return false;
    if ( not lookAt("/"+toString(arm)+"_grippermarker_expected", false)) 
        return false;

    if ( not lookAt("/"+toString(arm)+"_gripper", false)) 
        return false;
    if ( not lookAt("/"+toString(arm)+"_wrist", false)) 
        return false;
    if ( not lookAt("/"+toString(arm)+"_grippermarker_expected", false)) 
        return false;

    return true;
}

const bool ArmController::lookAtHand ( const Arms arm, const bool keep_tracking )
{
    return lookAt("/"+toString(arm)+"_grippermarker_expected", keep_tracking);
}

const bool ArmController::lookAtObservedHand ( const Arms arm, const bool keep_tracking )
{
    return lookAt("/"+toString(arm)+"_grippermarker_observed", keep_tracking);
}

const bool ArmController::lookAt( const std::string frame_id, const bool keep_tracking)
{
    rose20_gaze_controller::LookAtGoal goal;

    goal.target_point.header.frame_id   = frame_id;
    goal.target_point.point.x           = 0;
    goal.target_point.point.y           = 0;
    goal.target_point.point.z           = 0;
    goal.keep_tracking                  = keep_tracking;

    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::lookAtHand: Going to look at hand...");
    gaze_client_->sendGoal(goal);

    gaze_client_->waitForResult(ros::Duration(2.0));
    if ( gaze_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
    {
        ROS_DEBUG_NAMED(ROS_NAME, "ArmController::lookAtHand: Succeeded");
        return true;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::lookAtHand: Failed");
    return false;
}

void ArmController::togglePrerecordedVisualCorrection(const Arms arm, const bool switch_on)
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::togglePrerecordedVisualCorrection");
    ROS_DEBUG_NAMED(ROS_NAME, "PrerecordedVisualCorrection request for arm %i to be switched %s", arm, switch_on ? "on" : "off");
    setPrerecordedVisualCorrection(arm, switch_on);

    ROS_INFO_NAMED(ROS_NAME, "PrerecordedVisualCorrection for arm %i switched %s", arm, switch_on ? "on" : "off");
}    

void ArmController::resetRecordedVisualCorrection(const Arms arm)
{   
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::resetRecordedVisualCorrection");
    setRecordedVisualCorrection(arm, Pose());
    ROS_INFO_NAMED(ROS_NAME, "VisualCorrection for arm %i reset", arm);
}   

void ArmController::toggleVisualCorrection(const Arms arm, const bool switch_on)
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::toggleVisualCorrection");
    ROS_DEBUG_NAMED(ROS_NAME, "VisualCorrection request for arm %i to be switched %s", arm, switch_on ? "on" : "off");

    setVisualCorrection(arm, switch_on);

    ROS_INFO_NAMED(ROS_NAME, "VisualCorrection for arm %i switched %s", arm, switch_on ? "on" : "off");
}

//Show a visualization marker for a pose (The current goal)
void ArmController::publishTargetMarker(Arms arm, geometry_msgs::Pose pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/arms";
    marker.header.stamp = ros::Time();
    marker.ns = "arm_controller";
    marker.id =  (int)arm;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    ROS_DEBUG_NAMED(ROS_NAME, "Publishing marker to: X: %f, Y: %f, Z: %f. ", 
                                                                marker.pose.position.x, 
                                                                marker.pose.position.y, 
                                                                marker.pose.position.z);
    vis_pub_.publish( marker );
}

void ArmController::publishCorrectionVector(Arms arm, geometry_msgs::Pose correction_pose)
{
    visualization_msgs::Marker marker;

    std::string arm_frame = "";
    switch(arm)
    {
        case RIGHT:
            arm_frame = "right_arm";
            break;
        case LEFT:
            arm_frame = "left_arm";
            break;
    }

    marker.header.frame_id = "/" + arm_frame + "_grippermarker_expected";
    marker.header.stamp = ros::Time();
    marker.ns = "arm_controller";
    marker.id =  (int)arm+100;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;

    marker.points.push_back(origin); //0=start, 1=end
    marker.points.push_back(correction_pose.position); //0=start, 1=end
    
    marker.scale.x = 0.01; //scale.x is the shaft diameter, 
    marker.scale.y = 0.02; //and scale.y is the head diameter. 
    marker.scale.z = 0.0; //If scale.z is not zero, it specifies the head length.
    marker.color.a = 0.75;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    
    marker.frame_locked = false;

    ROS_DEBUG_NAMED(ROS_NAME, "Publishing arrow of: X: %f, Y: %f, Z: %f. ", 
                                                                marker.points[1].x, 
                                                                marker.points[1].y, 
                                                                marker.points[1].z);
    vis_pub_.publish( marker );
}

bool ArmController::CB_toggle(  arm_controller::toggle_visual_correction::Request  &req,
                                arm_controller::toggle_visual_correction::Response &res)
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_toggle");
    togglePrerecordedVisualCorrection(static_cast<Arms>(req.arm), req.apply_prerecorded_correction);
    toggleVisualCorrection(static_cast<Arms>(req.arm), req.apply_correction);
    return true;
}

bool ArmController::CB_reset(  arm_controller::reset_visual_correction::Request  &req,
                               arm_controller::reset_visual_correction::Response &res)
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_reset");
    resetRecordedVisualCorrection(static_cast<Arms>(req.arm));
    return true;
}

bool ArmController::CB_attach_item(arm_controller::set_item_attachment::Request &req, arm_controller::set_item_attachment::Response &res)
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_attach_item");
    attached_item_mutex_.lock();

    log_item_attachments();

    if(req.attached == req.ATTACH)
    {
        attached_items[(Arms)req.arm_index] = req.item_id;
        ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_attach_item attached item %s to arm index %i", req.item_id.c_str(), req.arm_index);
    }
    else
    {
        attached_items[(Arms)req.arm_index] = "";
        ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_attach_item detached item %s from arm index %i", req.item_id.c_str(), req.arm_index);
    }

    log_item_attachments();

    attached_item_mutex_.unlock();

    res.success = true;
    return true;
}

bool ArmController::CB_query_attached_items(arm_controller::get_item_attachment::Request &req, arm_controller::get_item_attachment::Response &res)
{
    ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_query_attached_items(arm_index=%i, item_id=%s)", req.arm_index, req.item_id.c_str());
    attached_item_mutex_.lock();

    log_item_attachments();

    Arms arm = (Arms)req.arm_index;

    if(!req.item_id.empty() && (arm != Arms::NONE && req.arm_index > 0)) //There is an item and actual arm specified
    {
        //Client wants to know if the given item is attached to the given arm.
        ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_query_attached_items: What is attached to what, for all items and arms?");
        for (auto& kv : attached_items)
        {
            if(kv.first == arm && kv.second == req.item_id)
            {
                res.arm_index = kv.first; //If we found the arm, the look up the item_id it maps to.
                res.item_id = kv.second; //If we found the arm, the look up the item_id it maps to.
                res.attached = true;
            }
        }
    }
    else if(!req.item_id.empty() && req.arm_index < 0) //If there is a specific item given and no arm
    {
        //Client wants to know to which arm the item is attached
        ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_query_attached_items: To which arm_index is item %s attached?", req.item_id.c_str());
        for (auto& kv : attached_items)
        {
            if(kv.second == req.item_id)
            {
                res.arm_index = kv.first; //If we found the arm, the look up the item_id it maps to.
                res.attached = true;
            }
        }
    }
    else if(arm != Arms::NONE && req.arm_index > 0) //If there is a specific arm given
    {
        //Client wants to know which item is attached to the arm_index
        ROS_DEBUG_NAMED(ROS_NAME, "ArmController::CB_query_attached_items: Which item is attached to arm_index %i?", req.arm_index);
        for (auto& kv : attached_items)
        {
            if(kv.first == arm)
            {
                res.item_id = kv.second; //If we found the arm, the look up the item_id it maps to.
                res.arm_index = (int)arm;
                res.attached = true;
            }
        }
    }

    attached_item_mutex_.unlock();

    return true;
}

void ArmController::log_item_attachments()
{
    for (const auto& kv : attached_items)
    {
        Arms arm = kv.first;
        std::string item = kv.second;
        ROS_DEBUG_NAMED(ROS_NAME, "ArmController: Arm index %i has item %s", (int)arm, item.c_str());
    }
}


Pose ArmController::getRecordedVisualCorrection( const Arms arm )
{
    error_pose_mutex_.lock();
    Pose pose = error_pose_[arm];
    error_pose_mutex_.unlock();

    return pose;
}

void ArmController::setRecordedVisualCorrection( const Arms arm, const Pose pose )
{
    error_pose_mutex_.lock();
    error_pose_[arm] = pose;
    error_pose_mutex_.unlock();
}

void ArmController::setVisualCorrection( const Arms arm, const bool on )
{
    apply_visual_correction_mutex_.lock();
    apply_visual_correction_[arm] = on;
    apply_visual_correction_mutex_.unlock();
}

bool ArmController::visualCorrectionSet( const Arms arm )
{
    apply_visual_correction_mutex_.lock();
    bool result = apply_visual_correction_[arm];
    apply_visual_correction_mutex_.unlock();

    return result;
}

void ArmController::setPrerecordedVisualCorrection( const Arms arm, const bool on )
{
    apply_rerecorded_visual_correction_mutex_.lock();
    apply_prerecorded_visual_correction_[arm] = on;
    apply_rerecorded_visual_correction_mutex_.unlock();
}

bool ArmController::prerecordedVisualCorrectionSet( const Arms arm )
{
    apply_rerecorded_visual_correction_mutex_.lock();
    bool result = apply_prerecorded_visual_correction_[arm];
    apply_rerecorded_visual_correction_mutex_.unlock();

    return result;
}
