# Simulating and Visualizing the Movement of a 6-Degrees of Freedom Robotic Arm
***

This project simulates and visualizes the movement of a 6-degrees of freedom robotic arm mounted on a self-propelled platform in a 3D virtual environment. The platform can move in any direction on the plane and rotate around its z-axis, while the arm receives hinge speed commands.

## Problem Description
***

The robotic system consists of a 6-degree freedom arm and a self-propelled platform on which the arm is mounted. One of the scenarios for using this robotic system is to pick up and move objects from a table to the platform. A cylindrical object is placed on a table, and the goal is to transport it to the platform.

The position and orientation of the frame of the object with respect to the inertial frame is given, as well as the position and orientation of the base frame of the robot with respect to the platform. The arm's model is provided in the file _lwr_create.p_, and the starting position of the robot joints is also given.

## Solution
***

The solution to this problem involves simulating the movement of the arm and visualizing it in a 3D virtual environment. This can be achieved using the Robotics Toolbox for MATLAB, which provides functions for kinematics, dynamics, and control of robotic systems. The 
analysis performed to calculate the trajectory is described in the report file. Also more details for the problem are provided inside the problem_definition file. This project was part of the _Robotics_ course. 

The simulation of the movement of the robotic arm is presented below.

<img src=robotic-arm.gif width="600" height="300" />