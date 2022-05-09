# limo_base

## About
This package contains the drivers and control nodes necessary to control the limo robot itself. The control node will communicate with the limo robot's mcu through a serial port.

## limo_base_node
### ROS Interfaces
| Published Topic | Type                 | Description                   |
| --------------- | -------------------- | ----------------------------- |
| `/odom`         | nav_msgs::Odometry   | Outputs robot's odometry data |
| `/limo_status`  | imo_base::LimoStatus | Outputs robot's status        |
| `/imu`          | sensor_msgs::Imu     | Outputs robot's imu data      |

| Subscribed Topic | Type                 | Description              |
| ---------------- | -------------------- | ------------------------ |
| `/cmd_vel`       | geometry_msgs::Twist | Control robot's movement |

| Parameter     | Type | Description                                             |
| ------------- | ---- | ------------------------------------------------------- |
| `port_name`   | str  | Serial port to robot controller.<br/>Default: "ttyTHS1" |
| `odom_frame`  | str  | Odometry frame id.<br />Default: "odom"                 |
| `base_frame`  | str  | Base frame id.<br/>Default: "base_link"<br/>            |
| `pub_odom_tf` | bool | Whether to publish odometry tf.<br />Default: false     |
| `use_mcnamu`  | bool | If using mecanum wheels.<br />Default: false            |

### Basic Usage
* Start the base node for limo
    ```
    roslaunch limo_base limo_start.launch
    ```


* Start the keyboard teleop node
    ```
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
    ```