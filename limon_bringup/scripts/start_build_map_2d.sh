#!/bin/bash

roslaunch build_map_2d revo_build_map_2d.launch &
sleep 3
roslaunch limon_bringup limon_start.launch & 
sleep 2