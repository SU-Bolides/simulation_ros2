# Bolide RViz Visualization

This repository contains the necessary files to visualize a bolide model in RViz2.  
**Note:** This is not a standalone ROS 2 package (no `package.xml` or `CMakeLists.txt`).The cloned folder contains only the URDF, launch, and RViz files. You must wrap it inside a ROS 2 package to build and use it properly.


## Setup Instructions

1. **Create a ROS 2 workspace:**

```bash
mkdir -p ~/bolide_ws/src
cd ~/bolide_ws/src
```


2. **Create a new empty package (e.g., bolide_visual):**

```bash
ros2 pkg create --build-type ament_cmake bolide_visual
cd bolide_visual
```

3. **Clone this repository inside the package:**

```bash
git clone https://github.com/SU-Bolides/simulation_ros2.git
```
You should now have the files located at:
```
~/bolide_ws/src/bolide_visual/simulation_ros2
```


4. **Modify `package.xml` and `CMakeLists.txt`:**

##### Add the following lines to your `package.xml`, after the `<buildtool_depend>` tag:

```xml
<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>rviz</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>gazebo_ros_control</exec_depend>
```

##### Add the following lines to the bottom of CMakeLists.txt, just before ament_package():

```cmake
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
```

5. **Build the workspace:**
```
cd ~/bolide_ws
colcon build --symlink-install
source install/setup.bash
```

6. **Launch the RViz visualization:**

```bash
ros2 launch simulation_ros2 display.launch.py
```

If everything works correctly, you should see a view like this:
 
![Bolide RViz View](https://raw.githubusercontent.com/SU-Bolides/simulation_ros2/main/rviz_simulation/rviz_output.png)

