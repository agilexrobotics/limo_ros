#!/bin/bash

roslaunch build_map_2d revo_build_map_2d.launch &
sleep 3
roslaunch limo_bringup limo_start.launch & 
sleep 2