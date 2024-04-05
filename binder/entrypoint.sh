#!/bin/bash

source ${PYCRAM_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch &
roslaunch --wait pycram ik_and_description.launch &

exec "$@"