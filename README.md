# limo_ros
This repository contains ROS packages for limo. 

<img src="limo_bringup/img/limo.jpg" width="640" height="208" /> 

## Packages
 
 
* limo_base: ROS wrapper for limo
* limo_bringup: launch and configuration files to start ROS nodes


## Build from source code
Clone the repository and catkin_make:
```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/agilexrobotics/limo_ros.git
    $ cd ..
    $ catkin_make
```


## Usage

* Start the base node for limo

    ```
    $ roslaunch limo_bringup limo_start.launch
    ```


* Start the keyboard tele-op node

    ```
    $ roslaunch limo_bringup limo_teleop_keyboard.launch
    ```
