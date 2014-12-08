/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/12/08
* 		- File created.
*
* Description:
*	Arm controller node. Starts the correct arm_controller.
* 
***********************************************************************************/
#include "arm_controller/arm_controller_node.hpp"

int main(int argc, char** argv)
{
    ros::NodeHandle n;
    ros::Rate r(100);

    // ROS_INFO_NAMED(ROS_NAME, "Starting main in arm_controller_node");

    pluginlib::ClassLoader<arm_controller_plugins::ArmControllerRobai> arm_controller("arm_controller_plugins", "arm_controller_base::ArmControllerBase");

    // get param
    std::string arm_controller_plugin = "arm_controller_plugins::ArmControllerRobai";

	try
	{
		boost::shared_ptr<arm_controller_plugins::ArmControllerRobai> triangle = arm_controller.createInstance(arm_controller_plugin);
	}
		catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}

    ros::init(argc, argv, "arm_controller");

    std::string ipAddress = "127.0.0.1";

    //! @todo MdL: Configurable?.
    /// Connect to Actin Server
    // ArmControllerRobaiBihand arm_controller("arms", n);
    // ArmControllerRobaiSinglehand arm_controller("arms", n);

    // ArmControllerDBMoveIt arm_controller("arms", n);

    // arm_controller.setCalibratingFunction(boost::bind(&armCalibration, _1 ));

    // arm_controller.initialize(ipAddress);
    // arm_controller.updateEndEffectorPoses();
    // arm_controller.publishEndEffectorPoses();
    // right_arm_controller.initialize(ipAddress);

    int i = 0;
    while (n.ok())
    {
        ros::spinOnce();
        r.sleep();

        // ROS_DEBUG_NAMED(ROS_NAME, "spinning..");
        if (i == 100)
        {
            // ROS_DEBUG_NAMED(ROS_NAME, "Updating poses!");
            // arm_controller.updateEndEffectorPoses();
            // arm_controller.publishEndEffectorPoses();
            i = 0;
        }
        i++;
    }

    // arm_controller.close();

    return 0;
}
