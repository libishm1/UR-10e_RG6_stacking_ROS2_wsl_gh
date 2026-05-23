#!/usr/bin/env bash
# Entrypoint for ur10e_rg6 ROS 2 Humble container.
# Sources ROS, optionally builds the workspace, sources the overlay.
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -d /workspace/src ] && [ "${SKIP_BUILD}" != "1" ]; then
    cd /workspace
    if [ ! -f /workspace/install/setup.bash ] || [ "${FORCE_BUILD}" = "1" ]; then
        echo "[entrypoint] colcon build (one-time)..."
        colcon build --symlink-install --event-handlers console_direct+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            --packages-skip moveit moveit_core moveit_ros_planning moveit_resources 2>&1 | tail -50 || true
    fi
fi

if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

exec "$@"
