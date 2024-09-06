#!/bin/bash
EXECUTED_ALREADY=$(rosparam get /roslaunch_executed_already)
# Check if the script has already been executed
if [ "$EXECUTED_ALREADY" == "true" ]; then
    exit 0
fi
# Get the parameter from the ROS parameter server
ROBOT_PARAM=$(rosparam get /nbparam_robots)

# Check if the parameter is set to 'pr2'
if [ "$ROBOT_PARAM" == "pr2" ]; then
    roslaunch ${PYCRAM_WS}/src/pycram/launch/pr2_standalone.launch > output.txt 2>&1
else
    echo "No parameter set"
fi
rosparam set /roslaunch_executed_already true