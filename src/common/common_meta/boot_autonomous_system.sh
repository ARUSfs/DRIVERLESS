#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/arus/workspaces/arus_ws/devel/setup.bash
# Ejecutar launchs
sleep 10

roslaunch can_c mission.launch
