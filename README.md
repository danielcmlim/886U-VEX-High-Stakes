# VEX High Stakes Robotics

C++ code developed for a **VEX V5 competition robot** for the VEX High Stakes game, featuring autonomous routines and driver-controlled operation.

## Overview

The project was designed to create a reliable competition robot that could perform consistently during autonomous and driver-controlled periods.

## Key Design Decisions

- **Odometry:** Used tracking wheels and an IMU to accurately estimate the robot's position and heading.
- **PID Control:** Used PID controllers to improve the accuracy and consistency of autonomous movement and turning.
- **LemLib:** Used for chassis control and odometry to simplify autonomous movement and positioning.
- **Motor Groups:** Grouped drivetrain motors to make movement control more consistent.
- **State-Based Control:** Used different mechanism states to make hood and arm control more predictable.
- **Pneumatics:** Used for mechanisms such as the mobile-goal clamp and doinker for quick actuation during matches.

## Autonomous

- Coordinate-based movement
- Automated mechanism control
- Odometry and PID-based positioning

## Driver Control

- Tank and arcade-style driving
- Intake and hood control
- Pneumatic mechanism control

## Technologies

- C++
- VEX V5
- PROS
- LemLib
- Odometry
- PID Control

## What I Learned

- C++ robotics programming
- Sensor and motor control
- Autonomous programming
- Odometry and PID tuning
- Debugging and testing physical hardware
