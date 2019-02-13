#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://roboraider:11311

roslaunch roboraider robot_run.launch
