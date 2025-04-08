#!/bin/bash

echo "Setting up workspace structure"
mkdir -p ~/workspace/ros/src
cd ~/workspace/ros
cd ~/workspace/ros/src

echo "Source ROS installation"
source /opt/ros/$ROS_DISTRO/setup.bash
echo "Cloning pycram and dependencies"
if [ "$ROS_VERSION" = "1" ]; then
  vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram.rosinstall
else
  vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram-ros2.rosinstall
fi

echo "Setting up virtual environment"
python3 -m venv pycram/pycram-venv --system-site-packages
source pycram/pycram-venv/bin/activate
echo "Installing python dependencies"
pip install -U pip
pip install -U setuptools
pip install -r pycram/requirements.txt
echo "Checking for dependencies of other ros packages"
cd ~/workspace/ros
echo "Building workspace"
if [ "$ROS_VERSION" = "1" ]; then
    catkin build
    source devel/setup.bash
    echo "source ~/workspace/ros/devel/setup.bash" >> ~/.bashrc
else
    colcon build --symlink-install
    source install/setup.bash
    echo "source ~/workspace/ros/install/setup.bash" >> ~/.bashrc
fi
echo "Successfully installed PyCRAM"