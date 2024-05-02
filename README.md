# ROSBot XL Kinematic Model Project

This repository contains all the code for the ROSBot XL Kinematic Model project developed in ROS2 and simulated using Gazebo Sim (Ignition). It includes packages for publishing wheel velocities, transforming these into robot movement commands, and following a complex figure-eight trajectory.

## Project Overview

The project aims to develop and implement a kinematic model for the ROSBot XL robot, enabling it to execute advanced robotic movements within a simulated environment. The main objectives include:
- Classifying the robot's movement capabilities.
- Developing and testing a kinematic model that represents the robot's movements.
- Implementing a sequence of complex movements, including a figure-eight trajectory.

## Packages

The repository includes the following packages:
- `wheel_velocities_publisher`: Publishes wheel speeds for the ROSBot XL.
- `kinematic_model`: Subscribes to wheel speeds and publishes Twist messages for robot control.
- `eight_trajectory`: Controls the robot to follow a figure-eight trajectory based on odometry.

## Getting Started

### Prerequisites
- ROS2
- Gazebo Sim (Ignition)

### Installation

1. Clone this repository into your ROS2 workspace src directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/rosbot-xl-kinematic-model.git
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

### Running the Simulation

1. Launch the ROSBot XL simulation:
   ```bash
   ros2 launch rosbot_xl_gazebo simulation.launch.py
   ```

2. In a new terminal, start the wheel velocities publisher:
   ```bash
   ros2 run wheel_velocities_publisher wheel_velocities_publisher
   ```

3. In another terminal, launch the kinematic model to convert wheel velocities into robot movement:
   ```bash
   ros2 launch kinematic_model kinematic_model.launch.py
   ```

4. To run the figure-eight trajectory:
   ```bash
   ros2 launch eight_trajectory eight_trajectory.launch.py
   ```

### Verification

- Make sure that the robot responds to `/cmd_vel` commands by using:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
  
- Stop the `wheel_velocities_publisher` node before launching the `kinematic_model.launch.py`.
