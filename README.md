# Object Collection using Computer Vision in ROS 2

## Overview

This ROS 2 package, `object_collection_py`, demonstrates object detection and collection using computer vision. The project involves a drone simulated in Gazebo that uses a camera to detect objects and a gripper to collect them. The objects are detected based on their color using OpenCV, and the drone approaches and collects the detected objects.

## Project Structure

object_collection_py/
├── CMakeLists.txt
├── config
│   └── controller.yaml
├── include
│   └── object_collection_py
│   └── CameraControllerPlugin.hh
├── launch
│   └── gazebo_launch.py
├── meshes
│   ├── 1345_prop_ccw.stl
│   ├── 1345_prop_cw.stl
│   ├── 5010Base.dae
│   ├── 5010Bell.dae
│   └── NXP-HGD-CF.dae
├── object_collection_py
│   ├── init.py
│   └── object_collection_node.py
├── package.xml
├── resource
│   └── object_collection_py
├── setup.cfg
├── setup.py
├── src
│   └── CameraControllerPlugin.cc
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── urdf
├── box.urdf
├── drone.urdf
├── dron_gemini.urdf.xacro
├── model.sdf
└── test.urdf


## Prerequisites

Ensure you have the following installed:
- ROS 2 Humble
- Gazebo
- OpenCV
- `cv_bridge`

## Installation

1. Clone this repository into your ROS 2 workspace:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/yourusername/object_collection.git
    ```

2. Install dependencies:

    ```bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro humble -y
    ```

3. Build the package:

    ```bash
    colcon build --packages-select object_collection_py
    ```

4. Source the workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Launch the Project

To launch the project, use the provided launch file:

```bash
ros2 launch object_collection_py gazebo_launch.py
