Tests on the Jaco arm using the wpi_jaco package
============
Date: 15-01-2015

Location: RWTH Hörn, Aachen

Tester: Mathijs de Langen

Introduction
------------

These tests show the usability of the current state of the following [rose_moveit_controller](https://github.com/RobotRose/rose_moveit_controller) software component.

Dependencies
------------
On which functions, nodes does this functionality directly rely?

* MoveIt!
* wpi_jaco

Test setup
----------

MoveIt is created solely for path planning. This is the point that this test is testing. 
Two ways of testing are used:
* Using rviz and interactive markers to send the arm to a specific goal.
* Use the interface of rose_moveit_controller.

##Tests and Results

### MoveIt! / wpi_jaco
There is a launch file *test_moveit_wpi.launch* in the launch directory for this test.

For this we start 

### rose_moveit_controller
There is a launch file *test_rose_moveit_controller.launch* in the launch directory for this test.