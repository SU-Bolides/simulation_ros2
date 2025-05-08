# Webots Simulation for Bolide Project

This repository provides a simulation environment using Webots and ROS 2 for controlling a TT02-based car model with various sensors and controllers.

## Installation

Follow the Webots installation procedure for Linux here: [Webots Installation Guide](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

Make sure to install the `webots_ros2` stack:

```bash
sudo apt install ros-${ROS_DISTRO}-webots-ros2
```

## Debugging Notes

If `/cmd_vel` or `/cmd_ackermann` does not appear, ensure your controller is correctly launched and its plugin path is registered in the URDF.

Webots `<extern>` controllers must match the `robot_name` in the `WebotsController()` block in the launch file.

Rebuild the project if necessary:

```bash
colcon build && source install/setup.bash
```

## Questions

For ROS 2–Webots integration questions, refer to the [webots_ros2 GitHub repository](https://github.com/cyberbotics/webots_ros2).