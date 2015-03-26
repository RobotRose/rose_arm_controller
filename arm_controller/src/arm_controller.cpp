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

namespace arm_controller_core {

ArmController::ArmController( std::string name, ros::NodeHandle n )
    : name_ ( name )
    , n_ ( n )
    , arm_controller_plugin_loader_("arm_controller_base", "arm_controller_base::ArmControllerBase")
    , sh_emergency_(SharedVariable<bool>("emergency"))
    , velocity_watchdog_("arm_velocity_watchdog", VELOCITY_TIMEOUT, boost::bind(&ArmController::CB_cancelVelocityForArms, this))
{
    ROS_INFO_NAMED(ROS_NAME, "Starting arm controller...");

    // Create server multiple clients
    createSMCs();

    // Load all parameters    
    loadArmParameters();

    // Load plugins filling arm_controllers
    loadArmPlugins();

    // Initialize all arm controllers
    initializeArmControllers();

    // attach_item_service_    = n_.advertiseService("/" + name_ + "/set_item_attachment",      &ArmController::CB_attach_item,  this);
    // query_attached_items_service_    = n_.advertiseService("/" + name_ + "/get_item_attachment",      &ArmController::CB_query_attached_items,  this);
    initializePublishersAndServices();

    // Register all shared variables
    registerSharedVariables();

    // Test arm movement. Only if enabled by config file
    testArmMovement();

    // Start all SMCs
    ROS_INFO_NAMED(ROS_NAME, "Starting all servers");
    set_position_smc_->startServer();
    set_velocity_smc_->startServer();
    set_gripper_width_smc_->startServer();
    set_wrench_smc_->startServer();

    ROS_INFO_NAMED(ROS_NAME, "Arm controller ready");
}

ArmController::~ArmController()
{
    closeAllArmControllers();
}

std::vector<std::string> ArmController::getArms()
{
    std::vector<std::string> arms;
    for (const auto& arm_controller : arm_controllers_)
        arms.push_back(arm_controller.first);

    return arms;
}

void ArmController::createSMCs()
{
    set_position_smc_ = new SMC_position(n_, name_+"/position",
            boost::bind(&ArmController::CB_receivePositionGoal, this, _1),
            boost::bind(&ArmController::CB_receivePositionCancel, this, _1)
    );
    set_velocity_smc_ = new SMC_velocity(n_, name_+"/velocity",
            boost::bind(&ArmController::CB_receiveVelocityGoal, this, _1),
            boost::bind(&ArmController::CB_receiveVelocityCancel, this, _1)
    );
    set_gripper_width_smc_ = new SMC_gripper(n_, name_+"/gripper_width",
            boost::bind(&ArmController::CB_receiveGripperGoal, this, _1),
            boost::bind(&ArmController::CB_receiveGripperCancel, this, _1)
    );    
    set_wrench_smc_ = new SMC_wrench(n_, name_+"/wrench",
            boost::bind(&ArmController::CB_receiveWrenchGoal, this, _1),
            boost::bind(&ArmController::CB_receiveWrenchCancel, this, _1)
    );
}

void ArmController::loadArmParameters()
{
    arm_plugins_.clear();

    if (n_.hasParam("plugins"))
    {
        XmlRpc::XmlRpcValue parameter_list;
        n_.getParam("plugins", parameter_list);
        for (int32_t i = 0; i < parameter_list.size(); ++i)
        {
            std::string plugin_name = static_cast<std::string>(parameter_list[i]["name"]);
            std::string plugin_type = static_cast<std::string>(parameter_list[i]["type"]);
            ROS_INFO("Using plugin %s with type %s", plugin_name.c_str(), plugin_type.c_str());

            arm_plugins_[plugin_name] = plugin_type;
        }
    }
    else
        ROS_ERROR("Could not retrieve plugins parameter");
}

void ArmController::loadArmPlugins()
{
    ROS_INFO_NAMED(ROS_NAME, "Loading %d plugins", (int)arm_plugins_.size());

    arm_controllers_.clear();
    for( const auto& arm_plugin : arm_plugins_ ) 
    {
        ROS_INFO_NAMED(ROS_NAME, "Loading %s", arm_plugin.first.c_str());
        try
        {
            boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;

            arm_controller = arm_controller_plugin_loader_.createInstance(arm_plugin.second);
            arm_controllers_[arm_plugin.first] = arm_controller;
        }
        catch(pluginlib::PluginlibException& ex)
        {
            ROS_ERROR_NAMED(ROS_NAME, "Arm controller was not added, the plugin failed to load: %s", ex.what());
        }
    }

    ROS_INFO_NAMED(ROS_NAME, "Loaded %d arm_controllers", (int)arm_controllers_.size());
}

void ArmController::initializeArmControllers()
{
    ROS_INFO_NAMED(ROS_NAME, "Initializing arm controllers...");
    for (const auto& arm_controller : arm_controllers_ )
        if ( not arm_controller.second->initialize(arm_controller.first) )
            ROS_ERROR("Could not initialize arm controller" );

    ROS_INFO_NAMED(ROS_NAME, "Done.");
}

void ArmController::initializePublishersAndServices()
{
    ROS_INFO_NAMED(ROS_NAME, "Initializing pub/sub");
    // attach_item_service_    = n_.advertiseService("/" + name_ + "/set_item_attachment",      &ArmController::CB_attach_item,  this);
    // query_attached_items_service_    = n_.advertiseService("/" + name_ + "/get_item_attachment",      &ArmController::CB_query_attached_items,  this);

    joint_state_publishers_.clear();
    for (const auto& arm_controller : arm_controllers_ )
    {
        ros::Publisher joint_state_pub = n_.advertise<sensor_msgs::JointState>(name_ + "/" + arm_controller.first + "/joint_states", 1);
        joint_state_publishers_[arm_controller.first] = joint_state_pub;
    }

    joint_state_timer_ = n_.createTimer(ros::Duration(0.0333), boost::bind(&ArmController::CB_updateJointStates, this));

    ROS_INFO_NAMED(ROS_NAME, "Initializing services");

    get_arms_service_ = n_.advertiseService("/" + name_ + "/get_arms", &ArmController::CB_get_arms, this);

    ROS_INFO_NAMED(ROS_NAME, "Done");
}

void ArmController::registerSharedVariables()
{
    //! @todo MdL: Uncomment.
    // Monitor the emergency button state
    // sh_emergency_.connect(ros::Duration(0.1));
    // sh_emergency_.registerChangeCallback(boost::bind(&ArmController::CB_emergency, this,  _1));
}

void ArmController::testArmMovement()
{
    ROS_INFO_NAMED(ROS_NAME, "Testing arm movement...");
    bool allow_first_movement;
    n_.param("/open_close_gripper_on_init", allow_first_movement, true);

    if (allow_first_movement)
        testMovementGrippers();

    ROS_INFO_NAMED(ROS_NAME, "Done");
}

void ArmController::closeAllArmControllers()
{
    for (const auto& arm_controller : arm_controllers_ )
        if ( not arm_controller.second->close() )
            ROS_ERROR("Could not close arm controller" ); //! @todo MdL: Add name of the controller / arm.
}

bool ArmController::stopArmMovement(const boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller)
{
    geometry_msgs::Twist twist;
    twist.linear.x  = 0.0;
    twist.linear.y  = 0.0;
    twist.linear.z  = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    velocity_watchdog_.stop();

    if(arm_controller->setEndEffectorVelocity(twist))
        return true;
    else
        return false;
}

void ArmController::testMovementGrippers()
{
    // Open and close the grippers
    for ( const auto& arm_controller : arm_controllers_ )
    {
        if ( not arm_controller.second->setGripperWidth(0.001))
            ROS_ERROR_NAMED(ROS_NAME, "Gripper could not close"); //0.02 m
        if ( not arm_controller.second->setGripperWidth(0.08)) //0.08 m
            ROS_ERROR_NAMED(ROS_NAME, "Gripper could not open");
    }
}

bool ArmController::armControllerExists(const std::string name)
{
    if ( arm_controllers_.find(name) == arm_controllers_.end())
    {
        ROS_ERROR_NAMED(ROS_NAME, "Arm controller \'%s\' does not exist", name.c_str());
        return false;
    }

    return true;
}

bool ArmController::getArmController(const std::string name, boost::shared_ptr<arm_controller_base::ArmControllerBase>& arm_controller)
{
    if ( not armControllerExists(name))
        return false;

    arm_controller = arm_controllers_[name];
    return true;
}
vector<string> ArmController::generateJointNames(const std::string arm_name, const int nr_of_joints)
{
    vector<string> names;
    for ( int i = 0 ; i < nr_of_joints ; i++ )
        names.push_back(arm_name + "_link" + std::to_string(i));

    return names;
}

void ArmController::updateJointStates()
{
    for ( const auto& arm_controller : arm_controllers_ )
    {
        sensor_msgs::JointState joint_states;
        joint_states.header.stamp  = ros::Time::now();
        joint_states.name          = generateJointNames(arm_controller.first, arm_controller.second->getNumberOfJoints());

        vector<double> joint_positions;
        vector<double> joint_velocities;
        vector<double> joint_efforts;

        // Get joint positions
        if ( arm_controller.second->getJointPositions(joint_positions) )
            joint_states.position = joint_positions;
        else
            ROS_WARN_ONCE("Could not retrieve joint positions for %s", arm_controller.first.c_str());

        // Get joint velocities
        if ( arm_controller.second->getJointVelocities(joint_velocities) )
            joint_states.velocity = joint_velocities;
        else
            ROS_WARN_ONCE("Could not retrieve joint velocities for %s", arm_controller.first.c_str());

        // Get joint efforts
        if ( arm_controller.second->getJointEfforts(joint_efforts) )
            joint_states.effort = joint_efforts;
        else
            ROS_WARN_ONCE("Could not retrieve joint efforts for %s", arm_controller.first.c_str());

        // Publish all information
        joint_state_publishers_[arm_controller.first].publish(joint_states);
    }
}

// Callback functions

void ArmController::CB_receivePositionGoal(const rose_arm_controller_msgs::set_positionGoalConstPtr& goal)
{
    if(sh_emergency_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Will not move arm due to emergency state.");
        return;
    }

    // get arm controller. 
    boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;
    if ( not getArmController(goal->arm, arm_controller))
        return;

    //! @todo MdL: Transform both to arm base link.
    Pose goal_pose =        goal->required_pose.pose;
    Twist goal_constraint = goal->constraint.twist;

    arm_controller->setConstraints(goal_constraint);

    //! @todo MdL [IMPL]: Result message.
    rose_arm_controller_msgs::set_positionResult result;
    set_position_smc_->sendServerResult(arm_controller->setEndEffectorPose(goal_pose), result );
}

void ArmController::CB_receivePositionCancel(SMC_position* smc)
{
    ROS_INFO_NAMED(ROS_NAME, "Position cancel received");
    rose_arm_controller_msgs::set_positionGoalConstPtr goal;
    if(smc->hasActiveGoal())
        if ( not smc->getCurrentGoal(goal) )
        {
            ROS_WARN_NAMED(ROS_NAME, "Could not get current goal. Not cancelling.");
            return;
        }
    else
    {
        ROS_WARN_NAMED(ROS_NAME, "Setting arm position cancelled, but there was no active goal.");
        return;
    }

    // Cancel arm controller
    boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;
    if ( not getArmController(goal->arm, arm_controller))
        return;

    arm_controller->cancel();
}

void ArmController::CB_receiveVelocityGoal(const rose_arm_controller_msgs::set_velocityGoalConstPtr& goal)
{
    if(sh_emergency_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Will not set arm velocity to emergency state.");
        return;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Received velocity request for arm %s", goal->arm.c_str());

    // get arm controller. 
    boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;
    if ( not getArmController(goal->arm, arm_controller))
        return;

    velocity_watchdog_.reset();

    //! @todo MdL [IMPL]: description.: Transform both to arm base link.
    Twist goal_twist        = goal->required_velocity.twist;
    Twist goal_constraint   = goal->constraint.twist;

    //! @todo MdL [IMPL]: description.: Constraint always Frame End Effector?.
    // arm_controller->setConstraints(goal_constraint);

    //! @todo MdL [IMPL]: Result message.
    rose_arm_controller_msgs::set_velocityResult result;
    set_velocity_smc_->sendServerResult(arm_controller->setEndEffectorVelocity(goal_twist), result );
}

void ArmController::CB_receiveVelocityCancel(SMC_velocity* smc)
{
    rose_arm_controller_msgs::set_velocityGoalConstPtr goal;
    if(smc->hasActiveGoal())
        if ( not smc->getCurrentGoal(goal) )
        {
            ROS_WARN_NAMED(ROS_NAME, "Could not get current goal. Not cancelling.");
            return;
        }
    else
    {
        ROS_WARN_NAMED(ROS_NAME, "Arm velocity cancelled, but there was no active goal.");
        return;
    }
    
    // get arm controller. 
    boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;
    if ( not getArmController(goal->arm, arm_controller))
        return;

    if ( not stopArmMovement(arm_controller))
        ROS_ERROR_NAMED(ROS_NAME, "Could not stop arm movement");
}

void ArmController::CB_receiveGripperGoal(const rose_arm_controller_msgs::set_gripper_widthGoalConstPtr& goal)
{
    if(sh_emergency_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Will not move arm gripper due to emergency state.");
        return;
    }

    // get arm controller. 
    boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;
    if ( not getArmController(goal->arm, arm_controller))
        return;

    rose_arm_controller_msgs::set_gripper_widthResult result;
    set_gripper_width_smc_->sendServerResult(arm_controller->setGripperWidth(goal->required_width), result );
}

void ArmController::CB_receiveGripperCancel(SMC_gripper* smc)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CB_receiveGripperCancel");
    rose_arm_controller_msgs::set_gripper_widthGoalConstPtr goal;
    ROS_DEBUG_NAMED(ROS_NAME, "Get active goal");
    if(smc->hasActiveGoal())
        if ( not smc->getCurrentGoal(goal) )
        {
            ROS_WARN_NAMED(ROS_NAME, "Could not get current goal. Not cancelling.");
            return;
        }
    else
    {
        ROS_WARN_NAMED(ROS_NAME, "Setting arm gripper width cancelled, but there was no active goal.");
        return;
    }

    if (goal == NULL)
        ROS_WARN_NAMED(ROS_NAME, "Last goal is NULL");

    ROS_DEBUG_NAMED(ROS_NAME, "Get arm controller %s", (goal->arm).c_str());
    boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;
    if ( not getArmController(goal->arm, arm_controller))
        return;

    arm_controller->cancel();
}

void ArmController::CB_receiveWrenchGoal(const rose_arm_controller_msgs::set_wrenchGoalConstPtr& goal)
{
    if(sh_emergency_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Will not set wrench due to emergency state.");
        return;
    }

    // get arm controller. 
    boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;
    if ( not getArmController(goal->arm, arm_controller))
        return;

    //! @todo MdL [IMPL]: Result message.
    rose_arm_controller_msgs::set_wrenchResult result;
    set_wrench_smc_->sendServerResult(arm_controller->setEndEffectorWrench(goal->required_wrench), result );
}

void ArmController::CB_receiveWrenchCancel(SMC_wrench* smc)
{
    rose_arm_controller_msgs::set_wrenchGoalConstPtr goal;
    if(smc->hasActiveGoal())
        if ( not smc->getCurrentGoal(goal) )
        {
            ROS_WARN_NAMED(ROS_NAME, "Could not get current goal. Not cancelling.");
            return;
        }
    else
    {
        ROS_WARN_NAMED(ROS_NAME, "Setting arm wrench cancelled, but there was no active goal.");
        return;
    }

    boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;
    if ( not getArmController(goal->arm, arm_controller))
        return;

    arm_controller->cancel();
}

void ArmController::CB_cancelVelocityForArms()
{
    // Stop all arm velocities
    for ( const auto& arm_controller : arm_controllers_ )
        if ( not stopArmMovement(arm_controller.second))
            ROS_ERROR_NAMED(ROS_NAME, "Could not stop arm movement");
}

void ArmController::CB_emergency(const bool& emergency)
{
    if(emergency)
        for ( const auto& arm_controller : arm_controllers_ )
            arm_controller.second->emergencyStop();
    else
        for ( const auto& arm_controller : arm_controllers_ )
            arm_controller.second->resetEmergencyStop();
}

void ArmController::CB_updateJointStates()
{
    updateJointStates();
}

bool ArmController::CB_get_arms(rose_arm_controller_msgs::get_arms::Request &req,
                                rose_arm_controller_msgs::get_arms::Response &res )
{
    for ( const auto& arm_controller : arm_controllers_)
        res.arms.push_back(arm_controller.first);
}

}; // namespace