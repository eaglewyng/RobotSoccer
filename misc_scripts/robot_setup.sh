#!/bin/bash

# trap ctrl-c
trap ctrl_c INT
function ctrl_c() {
    echo $vek_pid
    kill -9 $vek_pid
}

# start ros
roscore > /dev/null &

# start prediction node
cd ~/RobotSoccer/robot/motion
source devel/setup.bash
rosrun robot_soccer prediction_node.py _robot:=$1 > /dev/null &

# start Vektory
rosrun robot_soccer Vektory.py &
vek_pid=$!

wait $vek_pid
