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
#ifndef ARM_CONTROLLER_KINOVA_WPI_HPP
#define ARM_CONTROLLER_KINOVA_WPI_HPP

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <mutex>  
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <tf/tf.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group_interface/move_group.h>

#include <visualization_msgs/Marker.h>

#include "arm_controller_base/arm_controller_base.hpp"

#include "control_msgs/GripperCommand.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"
#include "control_msgs/GripperCommandFeedback.h"
#include "control_msgs/GripperCommandResult.h"

#include "wpi_jaco_msgs/JacoFK.h"
#include "wpi_jaco_msgs/CartesianCommand.h"
#include "wpi_jaco_msgs/AngularCommand.h"
#include "wpi_jaco_msgs/GetCartesianPosition.h"

#define NR_JOINTS               6           // Jaco has three, Mico uses this software and sets properties of the third finger to 0.0
#define COLLISION_CHECK_TIMER   0.5         // collision updates in seconds
#define MAX_PLANNING_TRIES      20         //! @todo MdL [CONF]: Make this a configurable parameter.

namespace arm_controller_plugins {    

using geometry_msgs::Pose;
using geometry_msgs::Twist;
using geometry_msgs::Wrench;
using sensor_msgs::JointState;
using std::vector;

 /**
  * @brief Provides an interface to interact with each kind of arm.
  */
class ArmControllerKinova : public arm_controller_base::ArmControllerBase {
  public:
    typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction>       GripperClient;

    /**
    * @brief  Constructor
    */
    ArmControllerKinova();

    /**
    * @brief  Destructor
    */
    ~ArmControllerKinova();

    bool initialize( const std::string name = std::string() );
    
    /**
     * @brief When closing the arm controller
     * @details The arm has to shutdown
     * @return If it was successful
     */
    bool close();

    /**
     * @brief   Cancels the current interaction of the arm. 
     * @details Cancelling only stopts the current interaction. The arm will respond again to new input.
     * @return  If the interaction was successfully cancelled.
     */
    bool cancel();

    /**
     * @brief   Stops the current interaction of the arm. 
     * @details The arm wil not respond to new inputs until resetEmergencyStop() has been called.
     * @return  If the interaction was successfully stopped.
     */
    bool emergencyStop();

    /**
     * @brief   Resets the emergency state to allow for new inputs.
     * @details Does not do anything if there is no emergency state.
     * @return  If the emergency state was successfully recoverd.
     */
    bool resetEmergencyStop();

    int getNumberOfJoints();

    /**
     * @brief Retrieves the position of the end effector.
     * @details A stamped pose of the gripper tip.
     * @return The end effector pose.
     */
    bool getEndEffectorPose(Pose& pose);

    /**
     * @brief Sets the end effector pose.
     * @details The arm will move to this position.
     * 
     * @param end_effector_pose The required end effector pose.
     * @return If the action was successful.
     */
    bool setEndEffectorPose(const Pose& end_effector_pose);

    /**
     * @brief Retrieves the end effector velocity.
     * @details Twist msg.
     * @return The end effector velocity in ROS twist message.
     */
    bool getEndEffectorVelocity(Twist& twist);

    /**
     * @brief Sets the end effector velicity.
     * @details The arm will try to move in the velocity required.
     * 
     * @param velocity The required velocity.
     * @return Is the action was successful.
     */
    bool setEndEffectorVelocity(const Twist& velocity);

    bool getConstraints(Twist& twist);

    bool setConstraints(const Twist& constraint);

    bool resetConstraints();

    double getGripperWidth();

    bool setGripperWidth(const double required_width);

    bool getEndEffectorWrench(Wrench& wrench);

    bool setEndEffectorWrench(const Wrench& Wrench);

    bool getJointPositions(vector<double>&);

    bool setJointPositions(const vector<double>&);

    bool getJointVelocities(vector<double>&);

    bool setJointVelocities(const vector<double>&);

    bool getJointEfforts(vector<double>&);

    bool setJointEfforts(const vector<double>&);
    // bool hasMoveItInterface();

  protected:
    bool loadParameters();
    bool loadMoveitConfiguration();

    bool updatePlanningScene();
    bool addDummyRobot();

    bool setAngularJointValues(const vector<double>& values, const bool& position);
    void CB_joint_state_received(const sensor_msgs::JointState::ConstPtr& joint_state);

    bool inCollision();
    bool checkForCollisions();
    bool updateCollisions();

    bool showEndEffectorGoalPose( const geometry_msgs::Pose& pose );

    ros::NodeHandle     n_;
    std::string         name_;
    
    ros::Publisher      arm_cartesian_command_publisher_;
    ros::Publisher      arm_angular_command_publisher_;
    ros::ServiceClient  get_cartesian_position_client_;
    ros::ServiceClient  planning_scene_service_client_;

    ros::Timer          collision_check_timer_;

    GripperClient*      gripper_client_;

    bool                emergency_;

    double              gripper_width_;

    std::mutex          joint_states_mutex_;
    JointState          joint_states_;
    ros::Subscriber     joint_state_sub_;
    bool                joint_states_initialized_;

    std::mutex          colision_mutex_;
    std::mutex          planning_scene_mutex_;
    bool                in_collision_;

    // Parameters
    std::string         arm_prefix_;
    double              max_gripper_width_;
    double              gripper_value_open_;
    double              gripper_value_closed_;
    int                 nr_fingers_;

    // MoveIt variables
    planning_scene::PlanningScene*                      planning_scene_;
    moveit::planning_interface::PlanningSceneInterface  planning_scene_interface_;
    moveit::planning_interface::MoveGroup*              move_group_;

    // Visualization
    ros::Publisher visualization_pub_;
};

} //namespace

#endif