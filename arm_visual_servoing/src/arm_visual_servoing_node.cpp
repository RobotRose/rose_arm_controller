/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/09/01
* 		- File created.
*
* Description:
*	This node provides visual servoing functionalities on the arms of the robot.
* 
***********************************************************************************/
#include "arm_visual_servoing_node.hpp"

int main(int argc, char **argv)
{   
    ros::init (argc, argv, "arm_visual_servoing");
    ros::NodeHandle n; 
    ros::Rate r(10);
 
    ArmVisualServoing* arm_visual_servoing = new ArmVisualServoing("arm_visual_servoing", n);

    while(n.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    delete arm_visual_servoing;

    return 0;
}