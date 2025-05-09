# COVAPSy Simulation Environment - Sorbonne Université 2025

This repository contains the simulation component of the COVAPSy course offered at Sorbonne Université in 2025.

## Overview

This simulation environment is designed to assist students in building, testing, and visualizing robotic control systems, with a focus on autonomous vehicle projects. It includes:

- URDF files for RViz visualization.
- Webots simulator integration with ROS 2 using dedicated controller plugins.
- Multiple STL models of various car bodies for visual customization.
- Custom ROS 2 nodes to control vehicles and simulate realistic autonomous behavior.

## Installation

To get started, first install Webots by following the official guide:  
[Webots Installation on Linux](https://cyberbotics.com/doc/guide/installing-webots)

## File Structure

- `launch/`: Launch files for simulation.
- `protos/`: Custom Webots PROTO files, including different STL car models.
- `resource/`: Contains the URDF and configuration files.
- `webot_simulation/`: Main ROS 2 package, including drivers and controllers.
- `worlds/`: Contains Webots world files.
- `test/`: Contains optional test files or tools.

## Setting up the interface between Webots and ROS2

Please refer to this tutorial [Webot_ROS2_interface](https://github.com/SU-Bolides/simulation_ros2/blob/main/webots_simulation/Webot_ROS2_interface.md) for more details.

## Adding a New World

To add a new Webots world:

1. Save the world file in the `worlds/` directory.
2. Register it in the `data_files` list in `setup.py`:

    ```python
    ('share/' + package_name + '/worlds', ['worlds/your_new_world.wbt'])
    ```

3. Set it as the default in the launch file:

    ```python
    DeclareLaunchArgument(
         'world',
         default_value='your_new_world.wbt',
         description='Choose one of the world files from the `/webot_simulation/world` directory'
    ),
    ```

## Adding a New Controller

If you create a new plugin controller that links Webots to ROS 2, update the plugin tag in the URDF file accordingly:

1. Set the plugin path in the URDF like this:
```xml
<plugin type="your_package.your_plugin_class"/>
```

2. Rebuild the project:

```bash
colcon build && source install/setup.bash
```

## Sensors and Devices

This simulation includes the following devices:

- RpLidar A2 (publishes to `/TT02_jaune/RpLidarA2/point_cloud`)
- IMU / Gyro
- Rear sonar sensor



## ROS 2 Nodes

1. **teleop.py**  
    Publishes `AckermannDrive` messages to the `/cmd_ackermann` topic. Ackermann messages are used because they are native to car models in Webots. This differs from the `low_level_ros2` approach, which uses `Twist` messages.

2. **obstacle_avoider.py**  
    Subscribes to the Lidar point cloud and performs basic obstacle avoidance. When an object is detected in front of the vehicle, it stops, reverses briefly, and turns.

## Maps Available

- `piste_enscopy.wbt`: A replica of the ENS 2023 track.
- `circular_piste.wbt`: A circular loop for testing.

To create your own maps, refer to the [Webots Map Creation Tutorial](https://cyberbotics.com/doc/guide/tutorial-4-appearance).

## Future Directions

This simulation provides a solid foundation for developing more advanced behaviors. Future goals include:

- Reinforcement Learning-based navigation
- Autonomous driving research

### Recommended Learning Resources

- TurtleBot3 DRL Navigation
- F1Tenth Reinforcement Learning

## Final Notes

Simulation differs from real-world scenarios. Be patient and exercise caution when transitioning to real hardware.

Enjoy exploring and driving!

For inspiration, refer to:

- [webots_ros2_tesla](https://github.com/cyberbotics/webots_ros2_tesla)
- [webots_ros2_turtlebot](https://github.com/cyberbotics/webots_ros2_turtlebot)
