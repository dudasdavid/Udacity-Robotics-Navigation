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
xterm  -e  " rosrun rviz rviz -d $(pwd)/src/Udacity-Robotics-Navigation/rvizConfig/home_service_robot.rviz" &
sleep 5
xterm  -e  " rosrun add_markers add_markers"