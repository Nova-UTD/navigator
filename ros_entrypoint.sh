#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [ -f /workspaces/overlay/install/setup.bash ]; then
  source /workspaces/overlay/install/setup.bash
fi
export DISPLAY="${DISPLAY:-host.docker.internal:0}"
exec "$@"
