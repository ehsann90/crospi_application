#!/usr/bin/env bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /spacenav/twist geometry_msgs/msg/Twist \
'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 20 --once
