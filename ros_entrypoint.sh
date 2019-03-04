#!/bin/bash
set -e

# setup ros2 environment
file="/opt/turtlebot3_ws/install/setup.bash"
if [ -f "$file" ]
then
  source "$file"
else
  source "/opt/turtlebot3_dependencies_ws/install/setup.bash"
fi

exec "$@"
