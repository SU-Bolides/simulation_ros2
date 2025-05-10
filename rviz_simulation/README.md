# Bolide RViz Visualization

This repository contains the necessary files to visualize a bolide model in RViz2.

> **Note**: This is now a valid ROS 2 Python package with `package.xml` and `setup.py`.

## Package Overview

The package includes:

- A `voiture.xacro` file describing the car.
- A `mesh` folder for visualization.
- A launch file to start RViz.
- A preconfigured RViz view (`bolide_viewer.rviz`).
- Setup files (`package.xml`, `setup.py`) for building the package.

## Setup Instructions

1. **Create a ROS 2 workspace**:
  ```bash
  mkdir -p ~/bolide_ws/src
  cd ~/bolide_ws/src
  ```

2. **Clone the package into your workspace**:
  ```bash
  git clone https://github.com/SU-Bolides/simulation_ros2.git
  ```

3. **Build the workspace**:
  ```bash
  cd ~/bolide_ws
  colcon build --packages-select rviz_simulation --symlink-install
  source install/setup.bash
  ```

4. **Launch RViz2 with the bolide viewer**:
  ```bash
  ros2 launch rviz_simulation rviz.launch.py
  ```

If everything works correctly, you should see a view like this:

![Bolide RViz View](https://raw.githubusercontent.com/SU-Bolides/simulation_ros2/main/rviz_simulation/rviz_output.png)

## Integration with Low-Level Car Simulation

Clone this visualization package after cloning the low-level control stack from [low_level_ros2](https://github.com/SU-Bolides/low_level_ros2).

Once the `low_level_ros2` repository is set up, follow its instructions to launch the car and its sensors.

Then, in a new terminal:

```bash
ros2 launch rviz_simulation rviz.launch.py
```

If everything is correctly configured, you should see the topics (e.g., `/scan`) published by the car directly in RViz2 — without modifying anything in this package.