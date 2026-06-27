# 2-DOF Planar Robotic Manipulator

This repository contains the software developed for a **2-degree-of-freedom planar robotic manipulator** capable of following arbitrary trajectories using inverse kinematics and PID position control plus gravitational compensation. The project integrates embedded programming, robot kinematics, motor characterization, and real-time trajectory visualization.

## Project Overview

The manipulator consists of two revolute joints driven by DC motors with magnetic quadrature encoders. A Python application generates the desired trajectory, computes the inverse kinematics, and communicates with the ESP32 microcontroller, which executes the motion using independent PID controllers for each joint.

The system is capable of reproducing parameterized trajectories while providing real-time feedback of the actual end-effector position based on encoder measurements. The robotic manipulator was build with the main objective of drawing distinct clover shapes.

## Repository Structure

### `src/main.cpp`

Firmware for the ESP32 developed using PlatformIO. It includes:

- Encoder acquisition
- Dual PID position control
- Motor actuation through an H-bridge L298N
- Serial communication with the host computer
- Execution of joint-space trajectories received from Python
- Real-time transmission of encoder data for visualization

### `inverse_kinematics.py`

Python application responsible for:

- Generating the desired Cartesian trajectory
- Computing inverse kinematics
- Automatically generating joint-space trajectories
- Sending trajectory data to the ESP32
- Receiving encoder feedback
- Computing forward kinematics
- Displaying the desired and actual trajectories in real time

Several trajectory parameters can be easily modified, including:

- Clover scale
- Rotation angle
- Cartesian position
- Number of repetitions

### `motor_characterization.ipynb`

Jupyter notebook used to characterize the motors and analyze their dynamic behavior. It contains:

- Experimental data processing
- Curve fitting
- Motor response analysis
- Controller tuning support

## Features

- 2-DOF planar manipulator control
- Inverse and forward kinematics
- Real-time trajectory generation
- Independent PID controllers for each joint
- Encoder-based position feedback
- Real-time trajectory visualization
- Configurable trajectory parameters
- Automatic communication between Python and ESP32

## Future Improvements

Potential extensions of the project include:

- Dynamic model implementation
- Torque-based control
- Simscape Multibody simulation
- Advanced controllers (Computed Torque, LQR, MPC)
- Improved mechanical design to reduce backlash and vibrations

## License

This repository was developed for academic purposes as part of a Mechatronics engineering project.