## start the motor driver

```shell
roslaunch limo_bringup limo_start.launch  
# motor driver, lidar, imu, robot pose ekf
```

## control Limo by keyboard

first launch the `limo_start.launch` above then launch the keyboard

```shell
# this will publish /cmd_vel topic
roslaunch limo_bringup limo_teletop_keyboard.launch
```

## start the move base for navigation

first launch the `limo_start.launch` above then launch the navigation

```shell
# this will launch the amcl and move_base node
roslaunch limo_bringup limo_navigation.launch
```
