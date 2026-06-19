#!/usr/bin/env bash
set -e

TASK_NAME="${1:-ala_reaching_v4_auto}"

source ~/ros2_ws/install/setup.bash

echo "Loading task parameters..."
ros2 service call /crospi_node/readTaskParameters crospi_interfaces/srv/TaskSpecificationString \
'{str: "{}"}'

echo "Loading robot specification..."
ros2 service call /crospi_node/readRobotSpecification crospi_interfaces/srv/TaskSpecificationFile \
'{file_path: "$[crospi_application]/robot_models/robot_specifications/ur10e.etasl.lua"}'

echo "Loading task specification: ${TASK_NAME}.etasl.lua"
ros2 service call /crospi_node/readTaskSpecificationFile crospi_interfaces/srv/TaskSpecificationFile \
"{file_path: \"\$[crospi_application]/task_specifications/libraries/ala_reaching_lib/task_specifications/${TASK_NAME}.etasl.lua\"}"

echo "Configuring Crospi..."
ros2 lifecycle set /crospi_node configure

echo "Activating Crospi..."
ros2 lifecycle set /crospi_node activate

echo "Crospi state:"
ros2 lifecycle get /crospi_node
