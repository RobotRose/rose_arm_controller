Tests on the Jaco arm using the wpi_jaco package in combination with our own MoveIt! interface
============
Date: 15-01-2015

Location: RWTH Hörn, Aachen

Tester: Mathijs de Langen (langen@robot-rose.nl)

Introduction
------------

These tests show the usability of the current state of the [rose_arm_controller](https://github.com/RobotRose/rose_arm_controller) software component.

Dependencies
------------
On which functions, nodes does this functionality directly rely?

* rose_moveit_controller
* wpi_jaco

Test setup
----------
This test inspects the basic functions of the rose generic arm controller with the Jaco arm

There is a launch file *test_all.launch* in the launch directory for this test.

The arm that is tested is referenced by the name **mico** (but it is in fact a Jaco arm).

## List of functions

### Action/Feedback/Result
Topic 							| Message Type     							| Description 
--------------------------------|:-----------------------------------------:|-----------
arm_controller/position     	| rose_arm_controller_msgs/set_position 	| Set the required position of the end effector
arm_controller/velocity 		| rose_arm_controller_msgs/set_velocity 	| Set the required velocity of the end effector
arm_controller/gripper_width 	| rose_arm_controller_msgs/set_gripper_width| Set the required gripper width 
arm_controller/wrench 			| rose_arm_controller_msgs/set_wrench 		| Set force and torque of the end effector

##Tests and Results

###arm_controller/position
Send a goal to set the position and orientation of the end effector of an arm.

All tests starting from a certain home position. 
The goal topic of jaco_arm/home_arm implemented in the wpi_jaco package.

Constraints are not implemented in the MoveIt! package, so we do not cover constraints at this time.

At this point, I do not know any valid cartesian positions for the arm. This is found out at the testing location.

###arm_controller/velocity
Send a goal to set the linear and angular velocity of the end effector of an arm.

All tests starting from a certain home position. The goal topic of jaco_arm/home_arm implemented in the wpi_jaco package.

Normally velocity command have set contraints (keeping the gripper level for instance). This particular field is asked by the actionlib server, but not implemented. As an extra test, we can try to find ou whether or not it is needed or not.

ID | Input values					| Expected Results 		| Measured values 	||
---|:------------------------------:|-----------------------|-------------------|-------
   | **required_velocity** [Twist]	| **position**			| **feedback**  	| **arm moved**
 0 | Home position 					| -						| yes/no			| yes/no 
 1 | linear.x = 0.1 				| home pos + *V* in x L	| yes/no			| yes/no
 2 | linear.y = 0.1 				| home pos + *V* in y L	| yes/no			| yes/no
 3 | linear.z = 0.1 				| home pos + *V* in z L	| yes/no			| yes/no
 4 | angualar.x = 0.1 				| home pos + *V* in x A	| yes/no			| yes/no
 5 | angualar.y = 0.1 				| home pos + *V* in y A	| yes/no			| yes/no
 6 | angualar.z = 0.1 				| home pos + *V* in z A	| yes/no			| yes/no

Here, *V* is a certain distance (time x velocity).

###arm_controller/gripper_width
Send a goal to set the gripper width of an arm.

ID | Input values			| Expected results 		|				| Measured values 		| 			|			    ||		
---|:----------------------:|-----------------------|---------------|-----------------------|-----------|---------------|----
   | **required_width** [m] | **actionlib succes**	| **end_width** | **actionlib succes**	| **width** | **feedback** 	| **arm moved** 
 1 | 0.0        			| true 					| 0.0      		| true/false			|			| yes/no		| yes/no		
 2 | 0.05        			| true 					| 0.05     		| true/false			|			| yes/no		| yes/no		
 3 | 0.10        			| true 					| 0.10 			| true/false			|			| yes/no		| yes/no		
 3 | 0.50 (wide)			| true 					| ? ~fully open	| true/false			|			| yes/no		| yes/no		

###arm_controller/wrench
Send a goal to force and torque of an arm.

Force and torque support is not yet implemented for the Jaco, hence this is not to be tested.