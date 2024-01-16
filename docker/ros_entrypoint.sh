#!/bin/bash
 
set -e

# Ros build
source "/opt/ros/noetic/setup.bash"


echo "==============VLA Unity Docker Env Ready================"

cd /root/catkin_ws

exec "$@"