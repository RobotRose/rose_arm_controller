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
    // , sh_emergency_(SharedVariable<bool>("emergency"))
    // , velocity_watchdog_("arm_velocity_watchdog", n, VELOCITY_TIMEOUT, boost::bind(&ArmController::cancelVelocityForArms, this))
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
    for ( int i = 0 ; i < number_of_arms ; i++)
    {
    	try
    	{
    		boost::shared_ptr<arm_plugins[i]> arm = arm_controller_plugin_loader_.createInstance(arm_plugins[i]);
    	}
    		catch(pluginlib::PluginlibException& ex)
    	{
    		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    	}

        // arm_controllers.push_back(arm_controller);
    }

    // Create SMC
    // smc_ = new SMC(n_, name_, boost::bind(&ArmController::CB_receiveGoal, this, _1, _2),
                              // boost::bind(&ArmController::CB_serverCancel, this, _1));
    // Init member variables
    // obstacle_map_ = map<Manipulators, Pose>();

    // attached_items = map<Arms, std::string>();

    // // Enable services / clients / publishers
    // toggle_service_         = n_.advertiseService("/" + name_ + "/toggle_visual_correction", &ArmController::CB_toggle, this);
    // reset_service_          = n_.advertiseService("/" + name_ + "/reset_visual_correction",  &ArmController::CB_reset,  this);
    // attach_item_service_    = n_.advertiseService("/" + name_ + "/set_item_attachment",      &ArmController::CB_attach_item,  this);
    // query_attached_items_service_    = n_.advertiseService("/" + name_ + "/get_item_attachment",      &ArmController::CB_query_attached_items,  this);

    // gaze_client_        = new GazeClient("gaze_controller", true);

    // vis_pub_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    // // Monitor the emergency button state
    // sh_emergency_.connect(ros::Duration(0.1));
    // sh_emergency_.registerChangeCallback(boost::bind(&ArmController::CB_emergencyCancel, this,  _1));

    ROS_INFO_NAMED(ROS_NAME, "Arm controller ready");
}

ArmController::~ArmController()
{
}

}; // namespace