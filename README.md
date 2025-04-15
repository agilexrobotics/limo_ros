# limo_ros
This repository contains ROS packages for limo. 

<img src="limo_description/img/limo.jpg" width="640" height="208" /> 

## Packages


* limo_base: ROS wrapper for limo
* limo_bringup: launch and configuration files to start ROS nodes
* limo_description: URDF model for limo 

## Build from source code

1. **Install YDLidar-SDK (Required Dependency)**  
   `ydlidar_ros_driver` depends on the YDLidar-SDK library.

   - **If you haven’t installed it or have an outdated version**:
     ```bash
     git clone https://github.com/YDLIDAR/YDLidar-SDK.git
     cd YDLidar-SDK
     # Follow the build instructions in the SDK's README.md:
     # https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md
     ```

   - **If you already have the latest SDK installed**, skip to the next step.

2. **Clone and build `limo_ros` and `ydlidar_ros_driver`**:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
   git clone https://github.com/agilerobotics/limo_ros.git
   cd ~/catkin_ws
   catkin_make


## Usage

* Start the base node for limo

    ```
    $ roslaunch limo_bringup limo_start.launch
    ```


* Start the keyboard teleop node

    ```
    $ roslaunch limo_bringup limo_teleop_keyboard.launch
    ```

    
