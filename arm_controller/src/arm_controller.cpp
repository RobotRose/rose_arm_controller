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
    , velocity_watchdog_("arm_velocity_watchdog", n, VELOCITY_TIMEOUT, boost::bind(&ArmController::CB_cancelVelocityForArms, this))
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

    // // Enable services / clients / publishers
    // toggle_service_         = n_.advertiseService("/" + name_ + "/toggle_visual_correction", &ArmController::CB_toggle, this);
    // reset_service_          = n_.advertiseService("/" + name_ + "/reset_visual_correction",  &ArmController::CB_reset,  this);
    // attach_item_service_    = n_.advertiseService("/" + name_ + "/set_item_attachment",      &ArmController::CB_attach_item,  this);
    // query_attached_items_service_    = n_.advertiseService("/" + name_ + "/get_item_attachment",      &ArmController::CB_query_attached_items,  this);

    // Register all shared variables
    registerSharedVariables();

    // Test arm movement. Only if enabled by config file
    testArmMovement();

    // Start all SMCs
    ROS_INFO_NAMED(ROS_NAME, "Starting all servers");
    set_position_smc_->startServer();
    set_velocity_smc_->startServer();
    set_gripper_width_smc_->startServer();

    ROS_INFO_NAMED(ROS_NAME, "Arm controller ready");
}

ArmController::~ArmController()
{
    closeAllArmControllers();
}

void ArmController::createSMCs()
{
    set_position_smc_ = new SMC_position(n_, name_+"/position",
            boost::bind(&ArmController::CB_receivePositionGoal, this, _1, _2),
            boost::bind(&ArmController::CB_receivePositionCancel, this, _1)
    );
    set_velocity_smc_ = new SMC_velocity(n_, name_+"/velocity",
            boost::bind(&ArmController::CB_receiveVelocityGoal, this, _1, _2),
            boost::bind(&ArmController::CB_receiveVelocityCancel, this, _1)
    );
    set_gripper_width_smc_ = new SMC_gripper(n_, name_+"/gripper_width",
            boost::bind(&ArmController::CB_receiveGripperGoal, this, _1, _2),
            boost::bind(&ArmController::CB_receiveGripperCancel, this, _1)
    );
}

void ArmController::loadArmParameters()
{
    arm_plugins_.clear();

    n_.param("/arm_controller/number_of_arms", nr_of_arms_, 1);
    for ( int i = 0 ; i < nr_of_arms_ ; i++ )
    {
        std::string plugin_name;
        std::string parameter = "/arm_controller/plugin"+boost::lexical_cast<std::string>(i);
        n_.param<std::string>(parameter, plugin_name, "arm_controller_plugins::ArmControllerRobai");
        arm_plugins_.push_back(plugin_name);
    }
}

void ArmController::loadArmPlugins()
{
    arm_controllers_.clear();
    for ( int i = 0 ; i < nr_of_arms_ ; i++)
    {
        try
        {
            boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;

            arm_controller = arm_controller_plugin_loader_.createInstance(arm_plugins_[i]);
            arm_controllers_.push_back(arm_controller);
        }
        catch(pluginlib::PluginlibException& ex)
        {
            ROS_ERROR("Arm controller was not added, the plugin failed to load: %s", ex.what());
        }
    }
}

void ArmController::initializeArmControllers()
{
    for (const auto& arm_controller : arm_controllers_ )
        if ( not arm_controller->initialize() )
            ROS_ERROR("Could not initialize arm controller" ); //! @todo MdL: Add name of the controller / arm.
}

void ArmController::registerSharedVariables()
{
    // Monitor the emergency button state
    sh_emergency_.connect(ros::Duration(0.1));
    sh_emergency_.registerChangeCallback(boost::bind(&ArmController::CB_emergency, this,  _1));
}

void ArmController::testArmMovement()
{
    bool allow_first_movement;
    n_.param("/arm_controller/allow_first_movement", allow_first_movement, true);

    if (allow_first_movement)
        testMovementGrippers();
}

void ArmController::closeAllArmControllers()
{
    for (const auto& arm_controller : arm_controllers_ )
        if ( not arm_controller->close() )
            ROS_ERROR("Could not close arm controller" ); //! @todo MdL: Add name of the controller / arm.
}

bool ArmController::stopArmMovement(const boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller)
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
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
        arm_controller->setGripperWidth(0.02); //0.02 m
        arm_controller->setGripperWidth(0.08); //0.08 m
    }
}

// Callback functions

void ArmController::CB_receivePositionGoal(const rose_arm_controller_msgs::set_positionGoalConstPtr& goal, SMC_position* smc)
{
    if(sh_emergency_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Will not move arm due to emergency state.");
        return;
    }

    //! @todo MdL: Transform both to arm base link.
    Pose goal_pose =        goal->required_pose.pose;
    Twist goal_constraint = goal->constraint.twist;

    arm_controllers_[goal->arm]->setContraints(goal_constraint);
    arm_controllers_[goal->arm]->setEndEffectorPose(goal_pose);
}

void ArmController::CB_receivePositionCancel(SMC_position* smc)
{
    rose_arm_controller_msgs::set_positionGoalConstPtr goal;
    if(smc->hasActiveGoal())
        goal = smc->getLastGoal();
    else
    {
        ROS_WARN_NAMED(ROS_NAME, "Setting arm position cancelled, but there was no active goal.");
        return;
    }

    //! @todo MdL: Cancel movement.
}

void ArmController::CB_receiveVelocityGoal(const rose_arm_controller_msgs::set_velocityGoalConstPtr& goal, SMC_velocity* smc)
{
    if(sh_emergency_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Will not set arm velocity to emergency state.");
        return;
    }

    velocity_watchdog_.reset();

    //! @todo MdL: Transform both to arm base link.
    Twist goal_twist = goal->required_velocity.twist;
    Twist goal_constraint = goal->constraint.twist;

    arm_controllers_[goal->arm]->setContraints(goal_constraint);
    arm_controllers_[goal->arm]->setEndEffectorVelocity(goal_twist);
}

void ArmController::CB_receiveVelocityCancel(SMC_velocity* smc)
{
    rose_arm_controller_msgs::set_velocityGoalConstPtr goal;
    if(smc->hasActiveGoal())
        goal = smc->getLastGoal();
    else
    {
        ROS_WARN_NAMED(ROS_NAME, "Arm velocity cancelled, but there was no active goal.");
        return;
    }

    if ( not stopArmMovement(arm_controllers_[goal->arm]))
        ROS_ERROR("Could not stop arm movement");
}

void ArmController::CB_receiveGripperGoal(const rose_arm_controller_msgs::set_gripper_widthGoalConstPtr& goal, SMC_gripper* smc)
{
    if(sh_emergency_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Will not move arm gripper due to emergency state.");
        return;
    }

    arm_controllers_[goal->arm]->setGripperWidth(goal->required_width);
}

void ArmController::CB_receiveGripperCancel(SMC_gripper* smc)
{
    rose_arm_controller_msgs::set_gripper_widthGoalConstPtr goal;
    if(smc->hasActiveGoal())
        goal = smc->getLastGoal();
    else
    {
        ROS_WARN_NAMED(ROS_NAME, "Setting arm gripper width cancelled, but there was no active goal.");
        return;
    }

    //! @todo MdL: Cancel setting gripper width.
}

void ArmController::CB_cancelVelocityForArms()
{
    // Stop all arm velocities
    for ( const auto& arm_controller : arm_controllers_ )
        if ( not stopArmMovement(arm_controller))
            ROS_ERROR("Could not stop arm movement");
}

void ArmController::CB_emergency(const bool& emergency)
{
    if(emergency)
        for ( const auto& arm_controller : arm_controllers_ )
            arm_controller->emergencyStop();
    else
        for ( const auto& arm_controller : arm_controllers_ )
            arm_controller->resetEmergencyStop();
}

}; // namespace