#!/bin/bash

source ${PYCRAM_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=${PYCRAM_WS}/src/pycram/binder/rviz_configs/pr2_config.json &
cp ${PYCRAM_WS}/src/pycram/binder/webapps.json ${PYCRAM_WS}/src/rvizweb/webapps/app.json 
python3 ${PYCRAM_WS}/src/pycram/demos/pycram_virtual_building_demos/setup_demo_manager.py

# Get the parameter from the ROS parameter server
ROBOT_PARAM=$(rosparam get /nbparam_robots)

# Check if the parameter is set to 'pr2'
if [ "$ROBOT_PARAM" == "pr2" ]; then
    roslaunch ${PYCRAM_WS}/src/pycram/launch/pr2_standalone.launch &
fi


xvfb-run exec "$@"
