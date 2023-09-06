#!/bin/bash

source ${PYCRAM_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=/binder/rvizweb-config.json &
roslaunch --wait pycram ik_and_description.launch &

jupyter lab workspaces import ${PYCRAM_WS}/src/pycram/binder/workspace.json

# Use xvfb virtual display when there is no display connected.
if [ -n "$DISPLAY" ]; then
  exec "$@"
else
  xvfb-run exec "$@"
fi