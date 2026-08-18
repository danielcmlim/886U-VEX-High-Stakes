# VEX High Stakes Robotics

C++ code for a VEX V5 competition robot built for the VEX High Stakes game. The project includes autonomous routines, driver control and mechanism control.

## Overview

This project was developed to create a reliable competition robot that performs consistently during both autonomous and driver-controlled periods.

The code combines drivetrain control, odometry, PID tuning and mechanism control to support accurate movement and repeatable match actions.

## Features

- Autonomous movement using coordinate-based routines
- Odometry using tracking wheels and an IMU
- PID control for more accurate driving and turning
- Driver control with tank and arcade drive options
- Intake, hood and arm control
- Pneumatic control for the mobile-goal clamp and doinker
- State-based mechanism control for more predictable hood and arm operation

## Design Decisions

- **Odometry**: Used tracking wheels and an IMU to estimate the robot's position and heading during autonomous routines.
- **PID control**: Used PID controllers to improve the accuracy and consistency of autonomous driving and turning.
- **LemLib**: Used LemLib for chassis control and odometry tools to simplify autonomous movement and positioning.
- **Motor groups**: Grouped drivetrain motors to provide more consistent movement and simplify motor control.
- **State-based control**: Used defined states for mechanisms such as the hood and arm to make their behaviour more predictable.
- **Pneumatics**: Used pneumatic mechanisms for the mobile-goal clamp and doinker to provide fast and reliable actuation during matches.

## Autonomous

- Coordinate-based movement
- Automated mechanism control
- Odometry-based positioning
- PID-controlled driving and turning

## Driver Control

- Tank drive and arcade drive options
- Intake, hood and arm control
- Pneumatic mechanism control

## Technologies

- [C++](https://isocpp.org/)
- [VEX V5](https://www.vexrobotics.com/v5)
- [PROS](https://pros.cs.purdue.edu/)
- [LemLib](https://lemlib.readthedocs.io/)
- Odometry
- PID control

## What I Learned

- Programming C++ for physical robotics systems
- Integrating sensors with motor control
- Developing autonomous routines for competition
- Setting up odometry and tuning PID controllers
- Designing state-based mechanism control
- Debugging and testing software with physical hardware in a competition environment
