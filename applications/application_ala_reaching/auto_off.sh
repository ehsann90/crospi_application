#!/usr/bin/env bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /ala/auto_cmd geometry_msgs/msg/Point \
'{x: 0.0, y: 0.0, z: 0.0}' \
-r 10
