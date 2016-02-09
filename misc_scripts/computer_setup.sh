#!/bin/bash

# start ros
roscore > /dev/null &

# start vision
cd ~/RobotSoccer/computer/vision
source devel/setup.bash
rosrun robot_soccer computer_vision > /dev/null &

# start command center
cd ~/RobotSoccer/robot/motion
source devel/setup.bash
rosrun robot_soccer VektoryCommandCenter.py