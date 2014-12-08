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

    ros::init(argc, argv, "arm_controller");

    arm_controller_core::ArmController arm_controller = new arm_controller_core::ArmController("arm_controller");

    while (n.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
