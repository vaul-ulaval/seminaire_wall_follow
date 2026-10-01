#!/usr/bin/env bash
set -eo pipefail

PKG='reactive_control'

if [[ ! -d "install/${PKG}" ]]; then
    echo -e "Building package ${PKG}..."
    colcon build --symlink-install
    echo -e "Package ${PKG} built successfully!"
fi

source install/setup.bash

printf "Ouvrez la connection ws://192.168.1.69:%s dans Foxglove.\n" "$FOXGLOVE_PORT"

ros2 run "$PKG" wall_follow_node
