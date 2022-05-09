# limo_bringup

## About
This package contains different launch/configuration/parameter files with each launch file launching different scenarios and environments when using the limo robot.

## Launch files

### limo_amcl
    Launches amcl for localization and robot_pose_ekf for odometry. Not intended to be launched by itself.
### limo_cartographer
    Launches cartographer related nodes for Simultaneous Localization and Mapping (SLAM). Not intended to be launched by itself.
### limo_gmapping
    Launches gmapping related nodes for laser-based SLAM. Not intended to be launched by itself.
### limo_navigation_ackerman(diff)
    Launches robot_pose_ekf for odometry, amcl for localization and move_base for navigation. Not intended to be launched by itself.
### limo_start
    Launches the limo robot control nodes and lidar nodes. This serves as the base configuration and is intended to be used in conjunction with other launch files.
### limo_teleop_keyboard
    Launches the teleop twist keyboard node to published /cmd_vel commands to the limo control nodes.