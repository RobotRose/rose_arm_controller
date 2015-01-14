Tests on the Jaco arm using the wpi_jaco package
============
Date: 15-01-2015

Location: RWTH Hörn, Aachen

Tester: Mathijs de Langen

Introduction
------------

These tests show the usability of the current state of the following [wpi_jaco](https://github.com/RIVeR-Lab/wpi_jaco/) software component.

Dependencies
------------
On which functions, nodes does this functionality directly rely?

* The (closed source) Kinova API.

Test setup
----------

On the [description](http://wiki.ros.org/wpi_jaco_wrapper) of the wpi_jaco package on the ROS website, is stated that the following functions are usable. These functions need to be tested accordingly.

This test inspects *only* the wpi_jaco package as a ROS node. The functions are tested with actionlib, rostopic publish or service calls.

## List of functions

### Action/Feedback/Result

Topic 							| Message Type     				| Description 
--------------------------------|:-----------------------------:|-----------
jaco_arm/fingers_controller     | control_msgs/GripperCommand 	| Gripper control using control_msgs
jaco_arm/home_arm 				| wpi_jaco_msgs/HomeArm 		| Move arm to the home position, or to a defined position via the home position

### Subscribed Topics

Topic 							| Message Type     				| Description 
--------------------------------|:-----------------------------:|-----------
jaco_arm/angular_cmd     		| wpi_jaco_msgs/AngularCommand 	| Send an angular command to the arm
jaco_arm/cartesian_cmd 			| wpi_jaco_msgs/CartesianCommand | Send a Cartesian command to the arm

### Published topics
Only for listening to output

Topic 							| Message Type     				| Description 
--------------------------------|:-----------------------------:|-----------
jaco_arm/joint_states     		| sensor_msgs/JointState 		| Publishes the current state of the arm and finger joints

### Services

Topic 							| Message Type     				| Description 
--------------------------------|:-----------------------------:|-----------
jaco_arm/get_cartesian_position | jaco_msgs/GetCartesianPosition| Read the Cartesian position from the arm

##Tests and Results

### jaco_arm/fingers_controller
Send a goal to the gripper to open, go half-way open and close.

ID | Input values			| Expected results 	|			| Measured values 	| 			| 				|
---|:----------------------:|-------------------| ----------|-------------------|-----------|---------------
   | command.position [m] 	| reached_goal		| position 	| reached_goal 		| position 	|  arm moved? 	|
 1 | 0.0        			| true 				| 0.0      	| true/false		|			| yes/no		|
 2 | 0.05        			| true 				| 0.05     	| true/false		|			| yes/no		|
 3 | 0.10        			| true 				| 0.10 		| true/false		|			| yes/no		|
 3 | 0.50 (wide)			| false 			| ?			| true/false		|			| yes/no		|

### jaco_arm/home_arm
Send a goal to move the arm to the homing position.

ID | Input values			| Expected Results 	| Measured values 	|   			|
---|:----------------------:|-------------------| ------------------| --------------
   | retract [bool]			| succes			| succes     		|  arm moved? 	|
 1 | false        			| true 				| true/false		| yes/no 		|

### jaco_arm/angular_cmd
Publish an angular command. Meaning: Sending joint angles. We do not test this at this time.

### jaco_arm/cartesian_cmd
Publish a cartesian command. Meaning: Sending cartesian velocities or positions to the arm.

The publish message is the following. We can split the test in position command, velocity command and finger command

Type 				| Variable 		| description 															|
--------------------|:-------------:|-----------------------------------------------------------------------
bool 				| position      | true for a position command, false for a velocity command 			|
bool 				| armCommand    | true if this command includes arm joint inputs						|
bool 				| fingerCommand | true if this command includes finger inputs 							|
bool 				| repeat        | true if the command should be repeatedly sent over a short interval 	|
geometry_msgs/Twist	| arm    		| position (m, rad) or velocity (m/s, rad/s) arm command 						|
float32[] 			| fingers   	| position (rad) or velocity (rad/s) finger command 					|

#### Position command (arm)
For this the *position* input is always set to *true*.
The variables armCommand and fingerCommand are both set to *false*, since this command does not include arm/finger inputs (whatever that might be).

ID | Input values			| Expected Results 	| Measured values 	|
---|:----------------------:|-------------------| ------------------
   | arm [Twist]					| position			| position     		|  
 1 | valid position 		|  					|  					|

#### Position command (fingers)
For this the *position* input is always set to *true*.
The variables armCommand and fingerCommand are both set to *false*, since this command does not include arm/finger inputs (whatever that might be).

ID | Input values			| Expected Results 	| Measured values 	|
---|:----------------------:|-------------------| ------------------
   | fingers [float32[]]	| position			| position     		|
 1 | [0.01, 0.01, 0.01]		|  					|  					|  
 2 | [0.1,0.1, 0.1]		 	|  					|  					|
 3 | [0.2,0.2, 0.2]		 	|  					|  					|

#### Velocity command (arm)
For this the *position* input is always set to *false*

The variables armCommand and fingerCommand are both set to *false*, since this command does not include arm/finger inputs (whatever that might be).

ID | Input values			| Expected Results 	| Measured values 	|
---|:----------------------:|-------------------| ------------------
   | arm [Twist]			| position			| position     		|  
 1 | valid position 		|  					|  					|

#### Velocity command (fingers)
For this the *position* input is always set to *false*

The variables armCommand and fingerCommand are both set to *false*, since this command does not include arm/finger inputs (whatever that might be).

ID | Input values			| Expected Results 	| Measured values 	|
---|:----------------------:|-------------------| ------------------
   | fingers [float32[]]	| position			| position     		|  
 1 | [0.01, 0.01, 0.01]		|  					|  					|  
 2 | [0.1,0.1, 0.1]		 	|  					|  					|
 3 | [0.2,0.2, 0.2]		 	|  					|  					|

### jaco_arm/joint_states
### jaco_arm/get_cartesian_position

ID | Test values       | Measured values   | Required values 
---|:-----------------:|-------------------| ---------------
 1 | parameters        | Measurements      | 
 2 | parameters        | Measurements      | 

Tests interesting for later
---------------------------

### jaco_manipulation
'jaco_manipulation' handles object manipulation actions including grasping and pickup actions. This is in an actionlib structure.

Topic 							| Message Type     				| Description 
--------------------------------|:-----------------------------:|-----------
jaco_arm/manipulation/grasp 	| wpi_jaco_msgs/ExecuteGrasp 	| Execute a grasp or release with the JACO gripper at a designated speed until the fingers can no longer move.
jaco_arm/manipulation/pickup	| wpi_jaco_msgs/ExecutePickup   | Execute a pickup action that lifts the end effector while applying a constant force to close the fingers, preventing objects from slipping.
