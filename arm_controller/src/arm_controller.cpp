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
    , set_position_smc_(n_, name_+"/position",
                                    boost::bind(&ArmController::CB_receivePositionGoal, this, _1, _2),
                                    boost::bind(&ArmController::CB_receivePositionCancel, this, _1)
                        )
    , set_velocity_smc_(n_, name_+"/velocity",
                                    boost::bind(&ArmController::CB_receiveVelocityGoal, this, _1, _2),
                                    boost::bind(&ArmController::CB_receiveVelocityCancel, this, _1)
                        )
    , set_gripper_width_smc_(n_, name_+"/gripper_width",
                                    boost::bind(&ArmController::CB_receiveGripperGoal, this, _1, _2),
                                    boost::bind(&ArmController::CB_receiveGripperCancel, this, _1)
                        )
    // , sh_emergency_(SharedVariable<bool>("emergency"))
    // , velocity_watchdog_("arm_velocity_watchdog", n, VELOCITY_TIMEOUT, boost::bind(&ArmController::cancelVelocityForArms, this))
{
    ROS_INFO_NAMED(ROS_NAME, "Starting arm controller...");

    // Load all parameters
    std::vector<std::string>    arm_plugins;
    
    n_.param("/arm_controller/number_of_arms", nr_of_arms_, 1);
    for ( int i = 0 ; i < nr_of_arms_ ; i++ )
    {
        std::string plugin_name;
        std::string parameter = "/arm_controller/plugin"+boost::lexical_cast<std::string>(i);
        n_.param<std::string>(parameter, plugin_name, "arm_controller_plugins::ArmControllerRobai");
        arm_plugins.push_back(plugin_name);
    }
    
    // Load plugins, add arm controllers
    arm_controllers_.clear();
    for ( int i = 0 ; i < nr_of_arms_ ; i++)
    {
        boost::shared_ptr<arm_controller_base::ArmControllerBase> arm_controller;
    	try
    	{
    		arm_controller = arm_controller_plugin_loader_.createInstance(arm_plugins[i]);
    	}
    		catch(pluginlib::PluginlibException& ex)
    	{
    		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    	}

        arm_controllers_.push_back(arm_controller);
    }

    // Init member variables
    // obstacle_map_ = map<Manipulators, Pose>();

    // attached_items = map<Arms, std::string>();

    // // Enable services / clients / publishers
    // toggle_service_         = n_.advertiseService("/" + name_ + "/toggle_visual_correction", &ArmController::CB_toggle, this);
    // reset_service_          = n_.advertiseService("/" + name_ + "/reset_visual_correction",  &ArmController::CB_reset,  this);
    // attach_item_service_    = n_.advertiseService("/" + name_ + "/set_item_attachment",      &ArmController::CB_attach_item,  this);
    // query_attached_items_service_    = n_.advertiseService("/" + name_ + "/get_item_attachment",      &ArmController::CB_query_attached_items,  this);

    // // Monitor the emergency button state
    // sh_emergency_.connect(ros::Duration(0.1));
    // sh_emergency_.registerChangeCallback(boost::bind(&ArmController::CB_emergencyCancel, this,  _1));

    ROS_INFO_NAMED(ROS_NAME, "Arm controller ready");
}

ArmController::~ArmController()
{
}


// Callback functions

void ArmController::CB_receivePositionGoal(const rose_arm_controller_msgs::set_positionGoalConstPtr& goal, SMC_position* smc)
{

}

void ArmController::CB_receivePositionCancel(SMC_position* smc)
{

}

void ArmController::CB_receiveVelocityGoal(const rose_arm_controller_msgs::set_velocityGoalConstPtr& goal, SMC_velocity* smc)
{

}

void ArmController::CB_receiveVelocityCancel(SMC_velocity* smc)
{

}

void ArmController::CB_receiveGripperGoal(const rose_arm_controller_msgs::set_gripper_widthGoalConstPtr& goal, SMC_gripper* smc)
{

}

void ArmController::CB_receiveGripperCancel(SMC_gripper* smc)
{

}


}; // namespace