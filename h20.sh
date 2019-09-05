#!/bin/bash
gnome-terminal -x bash -c  "roslaunch '/home/leo/h20_navigation/src/Robotmovement/launch/H20base_player_basic.launch';exec bash;"& sleep 1;

gnome-terminal -x bash -c  "roslaunch '/home/leo/h20_navigation/src/Robotmovement/launch/H20gmap.launch';exec bash;"& sleep 1;

gnome-terminal -x bash -c  "roslaunch '/home/leo/h20_navigation/src/TrajectorPlanner3/launch/trajectory.launch';exec bash;"& sleep 1;

gnome-terminal -x bash -c  "rosrun rviz rviz;exec bash;"

