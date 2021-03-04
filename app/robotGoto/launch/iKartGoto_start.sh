#!/bin/sh

roscore & 
sleep 2
$IKART_ROS_BRIDGE/bin/iKartRosBridge &
sleep 2
$IKART_ROS_BRIDGE/launch/load_map.sh $IKART_ROS_BRIDGE/launch/sestri_map &
sleep 2
roslaunch $IKART_ROS_BRIDGE/launch/ikart_localize.launch &
sleep 2
roslaunch $IKART_ROS_BRIDGE/launch/rviz_navigate.launch &
sleep 2
iKartGoto

