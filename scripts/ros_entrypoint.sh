#!/bin/bash
set -e

# setup ros environment
if [ ! -d "/opt/ros/melodic" ]; then
    source "/opt/ros/kinetic/setup.bash"
else 
    source "/opt/ros/melodic/setup.bash"
fi

exec "$@"
