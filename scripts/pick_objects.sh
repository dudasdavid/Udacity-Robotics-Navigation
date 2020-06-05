#!/bin/bash

# build the workspace
cd $(pwd)/../../..; catkin_make

source devel/setup.bash
#source /opt/ros/kinetic/setup.bash
export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/Udacity-Robotics-Navigation/worlds/MyWorld.world"
export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/src/Udacity-Robotics-Navigation/map/map.yaml"

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &    
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm  -e  " rosrun pick_objects pick_objects"