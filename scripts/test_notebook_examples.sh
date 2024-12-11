#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscd pycram/examples
rm -rf tmp
mkdir tmp
jupytext --to notebook *.md
mv *.ipynb tmp
cd tmp
roslaunch pycram ik_and_description.launch &
treon --thread 1 -v --exclude=migrate_neems.ipynb --exclude=improving_actions.ipynb