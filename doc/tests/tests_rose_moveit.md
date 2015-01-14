Tests on the Jaco arm using the rose_moveit_controller interface
============
Date: 15-01-2015

Location: RWTH Hörn, Aachen

Tester: Mathijs de Langen (langen@robot-rose.nl)

Introduction
------------

These tests show the usability of the current state of the [rose_moveit_controller](https://github.com/RobotRose/rose_moveit_controller) software component.

Dependencies
------------
* MoveIt!
* wpi_jaco

Test setup
----------

At this time MoveIt is used solely for path planning. Hence, we only test the path planning feature in this test.
Two ways of testing are used:
* Using rviz and interactive markers to send the arm to a specific goal.
* Use the interface of rose_moveit_controller.

The interface of the rose_moveit_controller is the following

Topic 							| Message Type     					| Description 
--------------------------------|:---------------------------------:|-----------
rose_moveit_controller/arm_goal | rose_moveit_controller/arm_goal 	| Send a goal (described next)

The goal message input is as follows

Type 		| Name 				| Description
------------|:-----------------:|---------
PoseStamped | goal_pose			| Cartesian goal pose of the end effector
float32 	| vx				| Cartesian x speed of the end effector
float32 	| vy				| Cartesian y speed of the end effector
float32 	| vz				| Cartesian z speed of the end effector
int32 		| control_mode		| 0 for position control, 1 for velocity control
int32 		| addObstacle		| not relevant
int32 		| gripperState_		| not used anymore
float32 	| offset			| not used anymore
float32 	| limit				| not used anymore
int32 		| code				| not used anymore
int32 		| server_indication	| not used anymore

##Tests and Results

### MoveIt! / wpi_jaco
There is a launch file *test_moveit_wpi.launch* in the launch directory for this test.

For this we start rviz with the interactive markers. We generate a random pose for the arm and plan to this position. We let MoveIt! create a plan and we animate this plan. If the plan does not go through the table, we store this pose (will be used in the next test).

### rose_moveit_controller

There is a launch file *test_rose_moveit_controller.launch* in the launch directory for this test.

The interface of the rose_moveit_controller is tested on planning to certain points. 

## Tests interesting for later

* Test the velocity control of the rose_moveit_controller.