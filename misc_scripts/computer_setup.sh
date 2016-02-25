#!/bin/bash

# start ros
roscore > /tmp/roscore_log.txt &

# start vision
cd ~/RobotSoccer/computer/vision
source devel/setup.bash
rosrun robot_soccer computer_vision > /tmp/vision_log.txt &

# start command center
cd ~/RobotSoccer/robot/motion
source devel/setup.bash
rosrun robot_soccer VektoryCommandCenter.py

