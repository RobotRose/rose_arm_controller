Tests on the Jaco arm using the wpi_jaco package
============
Date: 22-02-2015

Location: Rose BV, Horsten 1, Eindhoven

Tester: Mathijs de Langen (langen@robot-rose.nl)

Introduction
------------

These tests show the usability of the current state of the [wpi_jaco](https://github.com/RIVeR-Lab/wpi_jaco/) software component.

Dependencies
------------
* The (closed source) Kinova API.
* Installed Kinova drivers as stated in the Kinova manual.

Test setup
----------

On the [description](http://wiki.ros.org/wpi_jaco_wrapper) of the wpi_jaco package on the ROS website, is stated that the following functions are usable. These functions need to be tested accordingly.

This test inspects *only* the wpi_jaco package as a ROS node. The functions are tested with actionlib, rostopic publish or service calls.

There is a specific launch file *test_wpi_jaco.launch* in the launch directory for this test.

## List of functions

### Action/Feedback/Result

Topic 							               | Message Type     				| Description 
-----------------------------------------------|:------------------------------:|-----------
jaco_arm/fingers_controller/gripper (remapped from jaco_arm/fingers_controller) | control_msgs/GripperCommand    | Gripper control using control_msgs
jaco_arm/home_arm 				               | wpi_jaco_msgs/HomeArm 		    | Move arm to the home position, or to a defined position via the home position

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

Topic 							| Message Type     					| Description 
--------------------------------|:---------------------------------:|-----------
jaco_arm/get_cartesian_position | wpi_jaco_msgs/GetCartesianPosition| Read the Cartesian position from the arm

##Tests and Results

### jaco_arm/fingers_controller/gripper
Send a goal to the gripper to open, go half-way open and close.

ID | Input values			| Expected results 	|				| Measured values 	| 				||
---|:----------------------:|-------------------|---------------|-------------------|---------------|---------------
   | **command.position** [m]| **reached_goal**	| **position** 	| **reached_goal**	| **position** 	 | **arm moved** 
 1 | 0.0 (fully open)		| true 				| 0.0      		| true		        | __102.989__	 | yes		
 2 | 6400.0 (closed)        | true 				| 50.0     		| true      		| __0.209439510239__	| yes		
 3 | -10.0 (not allowed)    | false 			| ? 			| __true__         	| __111.8__		 | __yes__

The arm moves to the required input values. As we can see in this experiment the resulted values for the positions are not correct. Also, the result is retrieved almost immediately when sending a goal to the gripper. This can conclude why the resulting position is not correct. 

Secondly, in the last test we send a goal which is not possible (negative value). The reached goal result message is _true_ and the gripper moves towards this goal. This is also not expected.

Both issues have been seen when testing the Jaco arm. Issues are reported on [github](https://github.com/RIVeR-Lab/wpi_jaco/issues/19) and can probably be merged into our fork when the bug is fixed.

### jaco_arm/home_arm
Send a goal to move the arm to the homing position.

ID | Input values			| Expected Results 	| Measured values 	||
---|:----------------------:|-------------------|-------------------|---------------
   | **retract** [bool]		| **succes**		| **succes**    	| **arm moved**
 1 | false        			| true 				| true	         	| yes

 Works as expected.

### jaco_arm/angular_cmd
Publish an angular command. Meaning: Sending joint angles.

The message that is publishing in ROS is the following. We can split the test in the following commands:
* Position command for the arm; 
* Velocity command for the arm;
* Position command for the fingers; 
* Velocity command for the fingers;

Type 				| Variable 		| description 															
--------------------|:-------------:|-----------------------------------------------------------------------
bool 				| position      | true for a position command, false for a velocity command 			
bool 				| armCommand    | true if this command includes arm joint inputs						
bool 				| fingerCommand | true if this command includes finger inputs 							
bool 				| repeat        | true if the command should be repeatedly sent over a short interval 	
float32[]           | arm    		| position (rad) or velocity (rad/s) arm command				
float32[]   		| fingers   	| position (rad) or velocity (rad/s) finger command			

#### Position command (arm)
For this the *position* input is always set to *true*.
The variables armCommand is set to *true*, the fingerCommand is set to *false* and repeat is set to *false*.

For this test we also monitor the topic jaco_arm/joint_states for information about the arm.

I did not write down the actual joint values I have sent and the resulting values given by the joint state publisher. However, I could conclude from the test that the joint angles reached their position within an angle of 0.03 radians. This is also verified by the code corresponding to this function (it sets the goal as reached when the angle is within 0.03 radians).

##### Observations

Streching the arm:
```
rostopic pub /jaco_arm/angular_cmd wpi_jaco_msgs/AngularCommand "position: true
armCommand: true
fingerCommand: false
repeat: false
joints: [0.0,3.14,3.14,0.0,0.0,0.0]
fingers: [0,0,0]" 
```

Limits for joint 2 and 3:
Joint 2 [0.8, 5.3]
Joint 3 [0.6, 5.6]

Continious joints do not rotate more than once. So (in my observation) when sending a high value (say 1000) it will map it to a value within -PI and PI. This is also verified by the joint states topic.

#### Velocity command (arm)
For this the *position* input is always set to *true*.
The variables armCommand is set to *true* and the fingerCommand is set to *false*.

For this test we also monitor the topic jaco_arm/joint_states for information about the arm.

Input of values of *arm* (below) are all zero, unless described differently.

ID | Input values			 | Expected Results 		| Measured values 	||
---|:-----------------------:|-----------------------|----------------------|-------
   | **arm** [Twist]		 | **position**			 | **position**  	    | **arm moved**
 1 | joints = [0,0,0,0,0,40] | -                     | -  				    | yes, shortly
 1 | joints = [0,0,0,0,0,-40] | -                    |                      | yes, shortly

We have seen that the *repeat* has to be set to *true*. Otherwise, the arm will not move. 

Secondly, the velocity command has to be sent continiously. This has been verified by Dadid Kent (mainterner of the package):
> Velocity control will only send a command for about 1/60th of a second, so you need to send commands continuously at about 60 Hz.  I assume this is a safety feature of the API, so that if you lose connection to the arm it will stop.

##### Observations

With 
```
  rostopic pub -r 10 /jaco_arm/angular_cmd
```
you send a command with a rate of 10Hz. Using this, the arm kept rotating.

The input of the joints for the arm should be values between 0 and 1 (where 0 is 0% speed and 1 is 100% speed). Higher values than 1 do not cause a different the angle speed.

#### Position command (fingers)
For this the *position* input is always set to *true*.

The variables armCommand is set to *false* and the fingerCommand is set to *true*.

I did not write down the actual finger values I have sent and the resulting values given by the joint state publisher. However, I could conclude from the test that the joint angles reached their position within an angle of 0.03 radians. This is also verified by the code corresponding to this function (it sets the goal as reached when the angle is within 0.03 radians).

##### Observations

Opening the fingers:
```
rostopic pub /jaco_arm/angular_cmd wpi_jaco_msgs/AngularCommand "position: true
armCommand: false
fingerCommand: true
repeat: false
joints: [0,0,0,0,0,0]
fingers: [0,0,0]" 
```

Closing the fingers:
```
rostopic pub /jaco_arm/angular_cmd wpi_jaco_msgs/AngularCommand "position: true
armCommand: false
fingerCommand: true
repeat: false
joints: [0,0,0,0,0,0]
fingers: [6400,6400,0]" 
```

#### Velocity command (fingers)
The test starts with the fingers closed (position [0.0, 0.0, 0.0])

For this the *position* input is always set to *false*

The variables armCommand is set to *false* and the fingerCommand is set to *true*.

The conclusions from this test are the same as for the arm: We have seen that the *repeat* has to be set to *true*. Otherwise, the arm will not move. Secondly, the velocity command has to be sent continiously.

### jaco_arm/cartesian_cmd
Publish a cartesian command. Meaning: Sending cartesian velocities or positions to the arm.

The message that is publishing in ROS is the following. We can split the test in the following commands:
* Position command for the arm; 
* Velocity command for the arm;
* Position command for the fingers; 
* Velocity command for the fingers;

Type 				| Variable 		| description 															
--------------------|:-------------:|-----------------------------------------------------------------------
bool 				| position      | true for a position command, false for a velocity command 			
bool 				| armCommand    | true if this command includes arm joint inputs						
bool 				| fingerCommand | true if this command includes finger inputs 							
bool 				| repeat        | true if the command should be repeatedly sent over a short interval 	
geometry_msgs/Twist	| arm    		| position (m, rad) or velocity (m/s, rad/s) arm command 				
float32[] 			| fingers   	| position (rad) or velocity (rad/s) finger command 					

#### Position command (arm)
For this the *position* input is always set to *true*.
The variables armCommand is set to *true*, the fingerCommand is set to *false* and repeat is set to *false*.

For this test we also monitor the topic jaco_arm/joint_states for information about the arm.

I did not write down the actual positions I have sent and the resulting values given by the joint state publisher. However, I could conclude from the test that the joint angles reached their position within an acuracy of 0.03. This is also verified by the code corresponding to this function (it sets the goal as reached when the position is within a distance of 0.03).

When the position sent is unreachable, the arm will move towards the goal and eventually stop. I do not know if this is wanted behaviour.

#### Velocity command (arm)
For this the *position* input is always set to *false*

The variables armCommand is set to *true*, the fingerCommand is set to *false* and repeat is set to *true*.

For this test we also monitor the topic jaco_arm/joint_states for information about the arm.

All tests starting from home position. Input of values of *arm* (below) are all zero, unless described differently.

ID | Input values			| Expected Results 		| Measured values 	||
---|:----------------------:|-----------------------|-------------------|-------
   | **arm** [Twist]		| **position**			| **position**  	| **arm moved**
 0 | linear.z = 1 			| -						| - 				| yes, shortly

#### Position command (fingers)
For this the *position* input is always set to *true*.

The variables armCommand is set to *false*, the fingerCommand is set to *true* and repeat is set to *true*.

ID | Input values			| Expected Results 	| Measured values 	||
---|:----------------------:|-------------------|-------------------|-----------
   | **fingers** [float32[]]| **position**		| **position**    	| **fingers moved**
 1 | [0.0, 0.0, 0.0]		| -   				| - 				| yes, fully opened
 2 | [50, 50, 50]			| -  				| -                 | yes, fully closed

Behaves as expected.

#### Velocity command (fingers)
For this the *position* input is always set to *false*.

The variables armCommand is set to *false*, the fingerCommand is set to *true* and repeat is set to *true*.

ID | Input values			| Expected Results 	| Measured values 	||
---|:----------------------:|-------------------|-------------------|--------------
   | **fingers** [float32[]]| **position**		| **position** 		| **fingers moved**
 0 | [50.0, 50.0, 50.0]     | -                 | -                 | yes, closing gripper 
 1 | [-50.0, -50.0, -50.0]  | -					| -					| yes, opening gripper

Command has to be continuously sent.

#### Velocity command (combined)

#### Position command (combined)
For this the *position* input is always set to *true*.
The variables armCommand is set to *true*, the fingerCommand is set to *true* and repeat is set to *false*.

``` 
rostopic pub /jaco_arm/angular_cmd wpi_jaco_msgs/AngularCommand "position: true
armCommand: true
fingerCommand: true
repeat: false
joints: [0.0,3.14,3.14,0.0,0.0,3.14]
fingers: [0,0,0]"
```
Rotates the last joint and opens the gripper.

``` 
rostopic pub /jaco_arm/angular_cmd wpi_jaco_msgs/AngularCommand "position: true
armCommand: true
fingerCommand: true
repeat: false
joints: [0.0,3.14,3.14,0.0,0.0,0.0]
fingers: [6400,6400,0]"
```
Rotates the last joint and opens the gripper.

### jaco_arm/joint_states
Listen to topic when the arm moves

*Not specifically tested*

### jaco_arm/get_cartesian_position
Move arm with velocity control. Request cartesian position.

*Has been empirically verified during testing.* 

Tests interesting for later
---------------------------

### jaco_manipulation
'jaco_manipulation' handles object manipulation actions including grasping and pickup actions. This is in an actionlib structure.

Topic 							| Message Type     				| Description 
--------------------------------|:-----------------------------:|-----------
jaco_arm/manipulation/grasp 	| wpi_jaco_msgs/ExecuteGrasp 	| Execute a grasp or release with the JACO gripper at a designated speed until the fingers can no longer move.
jaco_arm/manipulation/pickup	| wpi_jaco_msgs/ExecutePickup   | Execute a pickup action that lifts the end effector while applying a constant force to close the fingers, preventing objects from slipping.

Both tested shortly:
* grasp did not give the expected result. The gripper did not close fully and it did not open at all. When trying to open I got the message "Gripper already open. Grasp execution complete.". This problem has been reported to WPI, but no further action is taken here.
* pickup moves a certain distance in the z-direction. This is not feasable for Rose, since we mount the arm differently.

Additional issues
-------------------
* ~~The arm would not connect via an USB 3 port. No action has been taken on this point.~~ Not an issue with the Mico arm.
* The WPI driver crashes when the arm is not connected, or disconnected while operating. This has been reported on [github](https://github.com/RIVeR-Lab/wpi_jaco/issues/14).