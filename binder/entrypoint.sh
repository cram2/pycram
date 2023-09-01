#!/bin/bash

source ${PYCRAM_WS}/devel/setup.bash
roscore &
roslaunch --wait pycram ik_and_description.launch &

exec "$@"