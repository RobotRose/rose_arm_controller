Tests on the Jaco arm (overall)
============
Date: 15-01-2015

Location: RWTH Hörn, Aachen

Tester: Mathijs de Langen (langen@robot-rose.nl)

Introduction
------------

These tests show the usability of the current state of the following software components:
* [wpi_jaco](https://github.com/RIVeR-Lab/wpi_jaco/)
* MoveIt! in combination with the API created by Debjyoti [link](https://github.com/RobotRose/rose_moveit_controller)
* This package, which combines all the packages above

* (Optional, as a bonus) [ros-jaco](https://github.com/Kinovarobotics/jaco-ros)

Test setup
----------
The test setup at Aachen is a Jaco arm mounted on top of a table. The control will be directly done via the command line.
* actionlibs axclient or;
* publishers (via rostopic pub).