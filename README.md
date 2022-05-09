# limo_ros

## About
This repository contains ROS packages for limo. 

<img src="limo_description/img/limo.jpg" width="640" height="208" /> 

## Packages
Each of the packages will have their own readmes that go further into detail on their usage.

* [limo_base](./limo_base): ROS wrapper for limo  
  This packages contains the driver and control node used to control the limo robot itself.
* [limo_bringup](./limo_bringup): launch and configuration files to start ROS nodes  
  This packae contains different launch files for convenient launching of different scenarios and tools.
* [limo_description](./limo_description): URDF model for limo  
  This package contains the URDF and meshes that makes up the limo model.
* [limo_gazebo_sim](./limo_gazebo_sim): Gazebo simulation for limo  
  This package contains the gazebo simulation environments for the limo robot.

## Building The Packages
Clone the packages into a catkin workspace and compile/source.
(the following instructions assume your catkin workspace is at: ~/catkin_ws/src)
```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/westonrobot/limo_ros.git
    cd ..
    catkin_make
    source devel/setup.bash
```


## Usage

* Start the base node for limo

    ```
    $ roslaunch limo_bringup limo_start.launch
    ```


* Start the keyboard teleop node

    ```
    $ roslaunch limo_bringup limo_teleop_keyboard.launch
    ```

    
