# PBL-Group5

## About

This project is the robot control code for a joint **Problem-Based Learning (PBL) robotics workshop** hosted at the **Sirindhorn International Institute of Technology (SIIT)**, run in collaboration with:

- **National Taipei University of Technology (NTUT)**
- **Osaka Institute of Technology (OIT)**
- **Tongji University**

Teams of students from all four institutions worked together to design, build, and program an autonomous robot for a timed cube-sorting competition task. My role on the team was **coding the robot's control program and the electrical wiring** of the motors, sensors, and actuators.


## Task Overview

The robot runs a fully autonomous mission that:

1. Drives from the start position to the conveyor belt
2. Pickup red/green cubes riding the belt, and drop them at a designated drop zone
3. Repeat until time runs out (2.5 minutes)

## My Contributions

- Wrote the full Arduino control program: the non-blocking task-state machine, drive/turn control with IMU + encoder feedback, sonar-based approach/stop logic, Pixy2-based cube and marker detection, and gripper/lift/gate actuator sequencing.
- Designed and wired the electrical system: motor driver connections, encoder wiring, sonar sensor wiring, IMU and Pixy2 I2C/serial connections, and servo power/signal wiring.
