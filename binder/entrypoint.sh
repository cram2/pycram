#!/bin/bash

source ${PYCRAM_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=${PYCRAM_WS}/src/pycram/binder/rviz_configs/pr2_config.json &
cp ${PYCRAM_WS}/src/pycram/binder/webapps.json ${PYCRAM_WS}/src/rvizweb/webapps/app.json 

xvfb-run exec "$@"
