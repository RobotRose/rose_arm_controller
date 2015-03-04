Tests on the Mico arm (overall)
============
Date: 23/02/2015

Location: Rose BV, Horsten 1, Eindhoven

Tester: Mathijs de Langen (langen@robot-rose.nl)

Introduction
------------

These tests show the usability of the current state of the following software components:
* [wpi_jaco](https://github.com/RIVeR-Lab/wpi_jaco/)
* [rose_moveit_controller](https://github.com/RobotRose/rose_moveit_controller), this package is an interface for MoveIt! created by Debjyoti
* [rose_arm_controller](https:://github.com/RobotRose/rose_arm_controller) for the Jaco/Mico arm. It uses wpi_jaco for the arm control and rose_moveit_controller (for path planning).

Still needs to be tested:
* [ros-jaco](https://github.com/Kinovarobotics/jaco-ros), has not been tested yet since there is no MoveIt! integration. 

Test setup
----------
The test setup at Rose consists of a Mico arm mounted not at the table top, but tilted 90 degrees. This way, it can act like an arm and shoulder. The control will be directly done via the command line.
* actionlibs axclient (rosrun actionlib axclient.pysrun), or;
* publishers (via rostopic pub), or; 
* servicecalls (rosservice).