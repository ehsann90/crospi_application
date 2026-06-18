#!/usr/bin/env bash
set -e

source ~/ros2_ws/install/setup.bash

ros2 service call /crospi_node/readTaskParameters crospi_interfaces/srv/TaskSpecificationString \
'{str: "{}"}'

ros2 service call /crospi_node/readRobotSpecification crospi_interfaces/srv/TaskSpecificationFile \
'{file_path: "$[crospi_application]/robot_models/robot_specifications/ur10e.etasl.lua"}'

ros2 service call /crospi_node/readTaskSpecificationFile crospi_interfaces/srv/TaskSpecificationFile \
'{file_path: "$[crospi_application]/task_specifications/libraries/ala_reaching_lib/task_specifications/ala_reaching_minimal.etasl.lua"}'

ros2 lifecycle set /crospi_node configure
ros2 lifecycle set /crospi_node activate
ros2 lifecycle get /crospi_node
