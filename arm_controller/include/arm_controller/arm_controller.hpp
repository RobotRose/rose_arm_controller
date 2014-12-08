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
#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP


#include <actionlib/client/simple_action_client.h>  //! @todo MdL: SMC?.
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "rose20_common/common.hpp"

#include "action_result_message.hpp"

#include "arm_controller/manipulateAction.h"
#include "arm_controller/manipulateGoal.h"
#include "arm_controller/manipulateResult.h"
#include "arm_controller/manipulateFeedback.h"
#include "arm_controller/reset_visual_correction.h"
#include "arm_controller/toggle_visual_correction.h"
#include "arm_controller/set_item_attachment.h"
#include "arm_controller/get_item_attachment.h"

#include "rose20_common/common.hpp"
#include "rose20_common/server_multiple_client/server_multiple_client.hpp"
#include "shared_variables/shared_variable.hpp"

#include "rose20_gaze_controller/LookAtAction.h"  //! @todo MdL: Coding guidelines (lower case filenames).
#include "rose20_gaze_controller/LookAtGoal.h"  //! @todo MdL: Coding guidelines (lower case filenames).

#include "visualization_msgs/Marker.h"

#include "rose_watchdogs/watchdog.hpp"

namespace arm_controller_core {

#define MAX_ERROR 0.10        // in meters
#define MAX_WAITING_TIME 4.5  // in seconds
#define VELOCITY_TIMEOUT 0.75  // seconds

using geometry_msgs::Pose;
using geometry_msgs::Vector3;
using geometry_msgs::Twist;
using std::find;
using std::map;
using std::vector;
using shared_variables::SharedVariable;

/**
 * This is the basic arm controller. It provides an interface to use a number of
 * arms in a generic way.
 */
class ArmController
{
  public:
    typedef boost::function<void(Pose* pose)> CalibratingFunction;
    typedef ServerMultipleClient<arm_controller::manipulateAction> SMC;
    typedef actionlib::SimpleActionClient<rose20_gaze_controller::LookAtAction>
        GazeClient;  //! @todo MdL: Make SMC.

    /**
     * Arms enumeration
     * All possible arms that can be used
     */
    enum Arms
    {
        RIGHT = 0,
        LEFT = 1,
        BOTH = 2,
        total,  //!< To count the number of possibilities of the arms
        NONE    //!< Used to define no arm
    };

    /**
     * Enum to string mapping
     * @param  arm Arm from Arms enumeration
     * @return     string representation of the arm
     */
    inline const std::string toString(Arms arm)
    {
        switch (arm)
        {
            case RIGHT:
                return "right_arm";
            case LEFT:
                return "left_arm";
            case BOTH:
                return "both_arms";
            default:
                return "[Unknown arm]";
        }
    }
    /**
     * End effector modes
     */
    enum EndEffectorMode
    {
        POINT_EE = 0,   //!< End effector is free to move in every direction
        FREE_SPIN = 1,  //!< End effector needs to be horizontally, but can move
                        // in the horizontal plane
        FRAME_EE = 2,   //!< End effector must keep its end effector in the same
                        // orientation
        count           //!< To count all possible end effectors (used in loops for
                        // instance)
    };

    /**
     * Possible manipulations on the arm
     */
    enum Manipulation
    {
        MOVE_TO_POSE,        //!< Move arm to a pose
        MOVE_RELATIVE_POSE,  //!< Move arm to a pose added to current pose
        MOVE_GRIPPER,        //!< Move the gripper (open/close/particular width)
        SET_JOINT_POSITIONS,  //!< Set the joint angles
        SET_VELOCITY,         //!< Set the velocity for the end effector
        SET_OBSTACLES,        //!< Change the location of obstacles
        REMOVE_OBSTACLES,
        CLEAR_OBSTACLES,
        // MOVE_SHAPE,           //!< Move the arm in a predefined kind of way
        // (line, circle)
        //! @todo MdL: Move shape? For MoveIt implementation.
    };

    /**
     * Available manipulators
     */
    enum Manipulators
    {
        ARM = 0,    //!< Arm manipulator
        BODY = 1,   //!< Body manipulator
        TABLE = 2,  //!< Table manipulator
        BOX = 3,    //!< Box manipulator
        last
    };

    enum ControlMode
    {
        POSITION,
        VELOCITY,
    };

    /**
     * Constructor
     * @param name Node name
     * @param n Node handle
     */
    ArmController(std::string name, ros::NodeHandle n);

    /**
     * Destructor
     */
    ~ArmController();

    /**
     * Cancel function. Propogate cancel classes that inherit from this class
     */
    virtual void cancel() = 0;

    void setCalibratingFunction(CalibratingFunction calibrating_function)
    {
        calibrating_function_ = calibrating_function;
    }

    virtual bool getEndEffectorPose(const Arms arm, Pose& pose) = 0;

  protected:
    /**
     * A required member function to move the arm to a specific pose
     * @param arm The arm that has to move
     * @param  pose The required pose
     * @param  mode The required end effector mode
     * @return      If the move was successful, a succes message, otherwise the
     * error message.
     */
    virtual ACTION_RESULT moveToPose(const Arms arm, const Pose pose,
                                     const EndEffectorMode mode) = 0;

    virtual ACTION_RESULT moveToRelativePose(const Arms arm, const Pose pose,
                                             const EndEffectorMode mode) = 0;

    /**
     * Moves the arm in a line towards a certain goal
     * @param  arm  Which arm to move
     * @param  mode The required end effector mode
     * @return      If the move was successful, a success message, otherwise the
     * error message
     */
    //! @todo MdL: Implementation.
    // virtual ACTION_RESULT moveInLine( const Arms arm, /* some other
    // parameters, */ const EndEffectorMode mode ) = 0;

    /**
     * A required member function to move the gripper of an arm to a percentage
     * of the total width
     * @param  arm   The arm that has to move its gripper
     * @param  width The required width in percentages of the total width
     * @return        If the move was successful, a succes message, otherwise
     * the error message.
     */
    virtual ACTION_RESULT moveGripper(const Arms arm, const double width) = 0;
    /**
     * A required member function to move the joint positions to a specific
     * angle
     * @param  arm The arm that has to move
     * @param  joint_positions The required joint positions in radians
     * @param  angle_tolerance The maximal difference between angle and actual
     * achieved angle
     * @return                 If the move was successful, a succes message,
     * otherwise the error message.
     */
    virtual ACTION_RESULT setJointPositions(
        const Arms arm, const vector<double> joint_positions,
        const double angle_tolerance) = 0;

    /**
     * A required member function to set the velocity of an arm
     * @param  arm   The arm that has to move
     * @param  twist The velocity of the arm
     * @return       If the move was successful, a succes message, otherwise the
     * error message.
     */
    virtual ACTION_RESULT setVelocity(const Arms arm, const Twist twist) = 0;

    /**
     * Move a certain obstacle (from the Manipulators list) to a certain pose in
     * the simulated environment.
     * @param  obstacle The obstacle to move
     * @param  pose     Where to move the obstacle
     * @return          Action result
     */
    virtual ACTION_RESULT setObstacle(const Manipulators obstacle,
                                      const Pose pose) = 0;

    /**
     * Publish the end effector poses of all arms
     */
    virtual void publishEndEffectorPoses() = 0;

    /**
     * Add a list of obstacles to the scene
     * @param  obstacle_map The list of obstacles with their poses to add to the
     * scene
     * @return              The result of the action
     */
    ACTION_RESULT addObstacles(const map<Manipulators, Pose> obstacle_map);

    /**
     * Remove an obstacle from the scene
     * @param  obstacle The obstacle to be removed
     * @return          The resyult of the action
     */
    ACTION_RESULT removeFromScene(const Manipulators obstacle);

    bool checkInputs(const Arms arm = Arms::RIGHT);

    /**
     * Callback that receives a goal (smc function)
     * @param goal The goal received
     * @param smc The smc of this node
     */
    void CB_receiveGoal(const arm_controller::manipulateGoalConstPtr& goal,
                        SMC* smc);

    /**
     * Callback that receives a cancel (smc function)
     * @param smc The smc of this node
     */
    void CB_serverCancel(SMC* smc);

    /**
     * Callback that is called when the shared variable sh_emercency_ is changed
     * @param new_value The new_value of the shared variable
     */
    void CB_emergencyCancel(const bool& new_value);

    /**
     * Send the proper result that goes with the retrieved result message
     * @param message The result message that belongs to the result
     */
    void sendResult( const ACTION_RESULT& message );
    void sendSuccess();
    void sendFail(const ACTION_RESULT action_result);
    bool resultSuccessful(const ACTION_RESULT action_result);

    /**
     * The move gripper services handles the goal MOVE_GRIPPER
     * @param arm          Which arm to move its gripper
     * @param width        The width of the gripper in percentages of the total
     * width
     * @param obstacle_map An optional list of obstacles in the environment
     */
    void moveGripperService(const Arms arm, const double width,
                            const map<Manipulators, Pose> obstacle_map =
                                map<Manipulators, Pose>());

    /**
     * The move pose service handles the goal MOVE_TO_POSE
     * @param arm          Which arm to move
     * @param pose         The pose it should reach
     * @param mode         The mode of the arm
     * @param obstacle_map An optional list of obstacles in the environment
     */
    void moveToPoseService(const Arms arm, Pose pose,
                           const EndEffectorMode mode,
                           const map<Manipulators, Pose> obstacle_map =
                               map<Manipulators, Pose>());

    void moveToRelativePoseService(const Arms arm, Pose pose,
                                   const EndEffectorMode mode,
                                   const map<Manipulators, Pose> obstacle_map =
                                       map<Manipulators, Pose>());

    /**
     * The set joints positions service handles the goal SET_JOINTS_POSITIONS
     * @param arm             For which arm to set the joint positions
     * @param joint_positions The joint positions in radians
     * @param obstacle_map    An optional list of obstacles in the environment
     */
    void setJointPositionsService(const Arms arm,
                                  const vector<double> joint_positions,
                                  const map<Manipulators, Pose> obstacle_map =
                                      map<Manipulators, Pose>());

    /**
     * The set velocity service handles the goal SET_VELOCITY
     * @param arm          For which arm to set the velocity
     * @param twist        The speeds to move
     * @param obstacle_map An optional list of obstacles in the environment
     */
    void setVelocityService(const Arms arm, const Twist twist,
                            const map<Manipulators, Pose> obstacle_map =
                                map<Manipulators, Pose>());

    /**
     * Removes a obstacle manipulator from the scene
     * @param manipulators The manipulators that should be removed. A vector
     * which represents the manipulator in enum Manipulators
     */
    void removeObstaclesService(const vector<int> manipulators);

    /**
     * Remove all obstacles from the scene
     */
    void clearObstaclesService();

    /**
     * Sets an obstacle from Manipulators to a certain pose (location and
     * orientation)
     * @param obstacle_map A map of manipulator with the pose requested
     */
    void setObstaclesService(const map<Manipulators, Pose> obstacle_map);

    /**
     * This service is called when no known goal is required in the goal
     */
    void actionNotDefinedService();

    /**
     * Publish a sphere of where the arms should move to
     * @param arm  For which arm to publish this marker
     * @param pose Where, relative to the arm's starting frame, to put the
     * sphere
     */
    void publishTargetMarker(Arms arm, Pose pose);

    /**
     * Show an arrow from the gripper of the arm to where the correction will
     * take it.
     * @param arm  For which arm to publish this marker
     * @param pose the position of the tip of the arrow. The base of the arrow
     * is at the *marker_expected-frame
     */
    void publishCorrectionVector(Arms arm, Pose pose);

    /**
     * Returns all key in a map of <Manipulators,Pose>
     * @param map The map of <Manipulators,Pose>
     * @return    A vector of all Manipulators stored in the map
     */
    vector<Manipulators> getKeys(const map<Manipulators, Pose> map);

    /**
     * Calculate the error between an expected and observed frame.
     * If the observated location of a frame is where it is expected, then the
     * error is zero.
     * Otherwise, it will be stored as an error_pose_
     * @param expected_frame The frame where the software thinks the arm marker
     * should be
     * @param measured_frame The observed frame
     * @param error_pose     The error as a pose
     * @return               Whether or not the function was executed
     * successfully. This function
     *                       can return false when no marker is observed.
     */
    bool calculateError(const std::string& expected_frame, 
                        const std::string& measured_frame, Pose& error_pose);

    /**
    * Apply an error correction (as obtained from ArmController::calculateError)
    * to a pose (uncorrected_pose).
    * When then moving to the corrected_pose that this method outputs, the frame
    * should be at the intended uncorrected_pose.
    * @param error_pose The error to add to the pose_to_correct
    * @param pose_to_correct The pose that should be corrected with error_pose
    * @return                Boolean value whether or not the correction was
    * issues successfully.
    */
    bool correctPoseWithError(const Pose& error_pose, Pose& pose_to_correct);

    /**
     * Measure and store the AR marker correction.
     * The error is stored in the member varaible error_pose_
     * @param arm The arm for which we want to find the visual correction
     * (error)
     * @return    Boolean value whether or not the correction was found and
     * successfully stored
     */
    bool recordVisualCorrection(const Arms arm);

    /**
     * Inspect hand looks at multiple points near the marker location. With a
     * moving image it is easier to
     * identify the marker
     * @param  arm Which arm to look at
     * @return     If it was successfull
     */
    const bool inspectHand(const Arms arm);
    /**
     * This function looks and the hand (where the pose is published) and after
     * that tries to observe and track it.
     * @param arm The arm to look at and track
     * @return    Boolean if the looking and tracking has been successfully
     * issued
     */
    const bool lookAtHandAndTrack(const Arms arm);

    /**
     * Looks at the arm gripper, where the pose of the gripper is published
     * @param arm The arm to look at
     * @param keep_tracking If the camera should track this point
     * @return              Boolean if the looking and tracking has been
     * successfully issued
     */
    const bool lookAtHand(const Arms arm, const bool keep_tracking = false);

    /**
     * Looks at the observed marker on an arm, where the observed marker is seen
     * @param arm The arm to look at
     * @param keep_tracking If the camera should track this marker
     * @return Boolean if the looking and tracking has been successfully issued
     */
    const bool lookAtObservedHand(const Arms arm,
                                  const bool keep_tracking = false);

    /**
    * Looks at a particular frame
    * @param arm The frame to look at
    * @param keep_tracking If the camera should track this frame
    * @return Boolean if the looking and tracking has been successfully issued
    */
    const bool lookAt(const std::string frame_id,
                      const bool keep_tracking = false);

    /**
     * Apply the visual correction that was previously recorded for this arm
     * @param arm The arm for with to apply the correction
     * @param pose The pose to correct
     */
    void applyPrerecordedVisualCorrection(const Arms arm, Pose& pose);

    /**
     * Apply the visual correction for this arm
     * @param arm  The arm for with to apply the correction
     * @param pose The pose to correct
     */
    void applyVisualCorrection(const Arms arm, Pose& pose);

    /**
     * Toggle whether the recorded correction (for that arm) is applied when the
     * next moveToPose is executed.
     * @param arm       Which arm to toggle prerecorded visual correction on
     * @param switch_on Toggle prerecorded visual correction on or off
     */
    void togglePrerecordedVisualCorrection(const Arms arm,
                                           const bool switch_on);

    /**
     * Reset the recorded correction (for that arm).
     * @param arm Which arm to reset.
     */
    void resetRecordedVisualCorrection(const Arms arm);

    /**
     * Toggle whether the next moveToPose will attempt to correct its pose.
     * @param arm       The arm to toggle visual correction on/off.
     * @param switch_on Toggle visual correction on or off
     */
    void toggleVisualCorrection(const Arms arm, const bool switch_on);

    /**
     * Toggle corrections on and off.
     * For example, when pre-grasping, apply (and record) the correction. So:
     * set request.apply_correction = true
     *     Then, when actually grasping, apply the correction we just recorded
     * with request.apply_prerecorded_correction = true
     *     and maybe request.apply_correction = true too.
     * @param req Request message for togglion visual correction. Contains for
     * which arm this message is, whether or not
     *            to do visual correction and prerecorded correction.
     * @param req Response message. Is empty.
     */
    bool CB_toggle(arm_controller::toggle_visual_correction::Request& req,
                   arm_controller::toggle_visual_correction::Response& res);

    /**
     * Reset accumulated corrections
     * @param req Request message for visual correction. Contains only which arm
     * to reset.
     * @param req Response message. Is empty.
     * @return    Boolean value whether the service call was successul
     */
    bool CB_reset(arm_controller::reset_visual_correction::Request& req,
                  arm_controller::reset_visual_correction::Response& res);

    /**
     * Grippers can hold objects and we must remember which gripper has which
     * item.
     * @param req Request message for item to gripper attachment
     * @param req Response message. Indicates success
     * @return    Boolean value whether the service call was successul
     */
    bool CB_attach_item(arm_controller::set_item_attachment::Request& req,
                        arm_controller::set_item_attachment::Response& res);

    /**
     * Grippers can hold objects and we must recall which gripper has which
     * item.
     * @param req Request message for item to gripper attachment
     * @param req Response message. Indicates success
     * @return    Boolean value whether the service call was successul
     */
    bool CB_query_attached_items(
        arm_controller::get_item_attachment::Request& req,
        arm_controller::get_item_attachment::Response& res);

    /**
      * Log some infor about the attached items
      */
    void log_item_attachments();

    /**
     * Setter for recorded visual correction
     * @param arm  For which arm to set the correction
     * @param pose The new recorded visual correction
     */
    void setRecordedVisualCorrection(const Arms arm, const Pose pose);

    /**
     * Getter for recorded visual correction
     * @param  arm For which arm to get the correction
     * @return     Recorded visual correction for arm
     */
    Pose getRecordedVisualCorrection(const Arms arm);

    /**
     * Setter for toggling on/off the prerecorded visual correction
     * @param arm For which arm to set the precorded visual correction on/off
     * @param on  Set the correction on of off
     */
    void setPrerecordedVisualCorrection(const Arms arm, const bool on);

    /**
     * Getter for the precorded visual correction
     * @param  arm For which arm to check the prerecorded visual correction
     * @return     Whether the visual correction is ON or OFF
     */
    bool prerecordedVisualCorrectionSet(const Arms arm);

    /**
     * Setter for toggling on/off the visual correction
     * @param arm For which arm to set the visual correction on/off
     * @param on  Set the correction on of off
     */
    void setVisualCorrection(const Arms arm, const bool on);

    /**
     * Getter for the visual correction
     * @param  arm For which arm to check the visual correction
     * @return     Whether the visual correction is ON or OFF
     */
    bool visualCorrectionSet(const Arms arm);

    void cancelVelocityForArms();

    map<Arms, bool> apply_prerecorded_visual_correction_;  //!< Wheter or not to apply a prerecorded correction to an arm
    map<Arms, bool> apply_visual_correction_;  //!< Whether or not to apply a
    // visual correction to an arm
    map<Arms, Pose> error_pose_;  //!< Current end effector error

    map<Manipulators, Pose> obstacle_map_;  //!< A map of obstacles with their
    // position, relative to the arms
    // base

    map<Arms, std::string> attached_items;  //!< A map that records which item
    // is in which arm

    std::string name_;         //!< Name of the node
    ros::NodeHandle n_;        //!< Node handle
    GazeClient* gaze_client_;  //!< Client to look at a specific point
    SMC* smc_;                 //!< Server multiple client

    ros::Publisher vis_pub_;  //!< Visualization for current goal

    ros::ServiceServer toggle_service_;  //!< Service to toggle visual
    // correction
    ros::ServiceServer reset_service_;  //!< Service to reset visual correction
    ros::ServiceServer attach_item_service_;  //!< Service remember which item
    // is in/attached to which gripper
    ros::ServiceServer query_attached_items_service_;  //!< Service to recall
    // which item is attached
    // to which gripper

    tf::TransformListener tf_;  //!< Transform listener for all transforms

    CalibratingFunction calibrating_function_;  //!< Static calibration function

    boost::mutex apply_rerecorded_visual_correction_mutex_;  //!< Mutex
    boost::mutex apply_visual_correction_mutex_;             //!< Mutex
    boost::mutex error_pose_mutex_;                          //!< Mutex
    boost::mutex attached_item_mutex_;                       //!< Mutex

    SharedVariable<bool> sh_emergency_;                      //!< Shared variable

    rose::Watchdog velocity_watchdog_;
}; // ArmController
}; //namespace
#endif  // ARM_CONTROLLER_HPP
