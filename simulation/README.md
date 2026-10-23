# Simulation

A ROS 2 and Gazebo-based simulation environment for a Kiwi omni-directional mobile robot.

## Requirements

The simulation requires the following environment:

| Software   | Version          |
| ---------- | ---------------- |
| **Ubuntu** | 22.04 LTS        |
| **ROS 2**  | Humble Hawksbill |
| **Gazebo** | Fortress         |
| **Python** | 3.10.12             |

> **Important:** ROS 2 Humble is required. Using a different ROS 2 distribution may result in package or dependency incompatibilities.

### Verify the Environment

Check the ROS 2 distribution:

```bash
echo $ROS_DISTRO
```

Expected output:

```text
humble
```

Check the Ubuntu version:

```bash
lsb_release -a
```

Check the Gazebo installation:

```bash
gazebo --version
```

---

## Dependencies

Install the required ROS 2 packages:

```bash
sudo apt update

sudo apt install \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-slam-toolbox
```

The simulation uses:

* **ROS 2** for communication, control, and system integration
* **Gazebo Fortress** for physics simulation and visualization
* **Xacro / URDF** for the robot description
* **ROS-Gazebo Bridge** for communication between ROS 2 and Gazebo
* **SLAM Toolbox** for mapping and localization

---

## Directory Structure

```text
simulation/
└── kiwi_robot/
    ├── config/
    │   └── slam_toolbox_params.yaml
    │
    ├── launch/
    │   ├── gazebo.launch.py
    │   └── slam.launch.py
    │
    ├── scripts/
    │   ├── autonomous_controller.py
    │   ├── navigation_controller.py
    │   └── omni_drive_node.py
    │
    ├── urdf/
    │   └── kiwi_robot.urdf.xacro
    │
    └── worlds/
        └── kiwi_world.sdf
```

### `config/`

Contains configuration files used by the simulation, including SLAM Toolbox parameters.

### `launch/`

Contains ROS 2 launch files for starting the simulation and SLAM pipeline.

### `scripts/`

Contains Python nodes for robot control, navigation, and omni-directional drive functionality.

### `urdf/`

Contains the Xacro-based robot description, including the robot's links, joints, sensors, and simulation properties.

### `worlds/`

Contains Gazebo world definitions used for the simulation environment.

---

## Setup

Clone the repository containing the simulation and navigate to the simulation workspace:

```bash
cd simulation
```

Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

Build the workspace:

```bash
colcon build --symlink-install
```

Source the newly built workspace:

```bash
source install/setup.bash
```

---

## Running the Simulation

### Launch Gazebo

```bash
ros2 launch kiwi_robot gazebo.launch.py
```

This launches the Gazebo environment and spawns the Kiwi robot.

### Launch SLAM

In a separate terminal, source the required environments:

```bash
source /opt/ros/humble/setup.bash
source simulation/install/setup.bash
```

Then launch SLAM:

```bash
ros2 launch kiwi_robot slam.launch.py
```

---

## ROS 2 Commands

Useful commands for inspecting the running simulation:

### List Nodes

```bash
ros2 node list
```

### List Topics

```bash
ros2 topic list
```

### Inspect a Topic

```bash
ros2 topic echo /<topic_name>
```

### Inspect Topic Information

```bash
ros2 topic info /<topic_name>
```

### List Available Packages

```bash
ros2 pkg list | grep kiwi
```

---

## Clean Build

If the workspace needs to be rebuilt from scratch:

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

The `build/`, `install/`, and `log/` directories are generated automatically by `colcon` and do not need to be manually downloaded or copied.

---

## Notes

* Use **Ubuntu 22.04 + ROS 2 Humble** for the intended environment.
* Ensure all required ROS 2 packages are installed before building.
* Source `/opt/ros/humble/setup.bash` before using ROS 2 commands.
* Source the local `install/setup.bash` after building the workspace.
* The simulation source files are contained within the `kiwi_robot` package.
