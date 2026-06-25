#!/usr/bin/env bash
source ~/ros2_ws/install/setup.bash

timeout 1 ros2 topic pub /spacenav/twist geometry_msgs/msg/Twist \
'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 20 || true --once

timeout 1 ros2 topic pub /ala/auto_cmd geometry_msgs/msg/Point \
'{x: 0.0, y: 0.0, z: 0.0}' -r 10 || true --once
