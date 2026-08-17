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

## Key Design Decisions

### Odometry

Tracking wheels and an IMU estimate the robot's position and heading during autonomous routines.

### PID Control

PID controllers improve the accuracy and consistency of autonomous driving and turning.

### LemLib

[LemLib](https://lemlib.readthedocs.io/) provides chassis control and odometry tools that simplify autonomous movement and positioning.

### Motor Groups

Drivetrain motors are grouped to provide more consistent movement control.

### State-Based Control

Mechanisms use defined states to make hood and arm behaviour more predictable.

### Pneumatics

Pneumatics operate match mechanisms such as the mobile-goal clamp and doinker for fast actuation.

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

- C++ programming for physical robotics systems
- Sensor integration and motor control
- Autonomous routine development
- Odometry setup and PID tuning
- Debugging and testing hardware in a competition environment
