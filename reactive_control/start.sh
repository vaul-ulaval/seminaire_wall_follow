#!/usr/bin/env bash
set -eo pipefail

colcon build --symlink-install
source install/setup.bash

ros2 run reactive_control wall_follow_node

printf "Ouvrez la connection ws://192.168.1.69:%s dans Foxglove.\n" "$FOXGLOVE_PORT"
