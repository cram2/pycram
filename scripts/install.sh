#!/bin/bash

mkdir -p ~/workspace/ros/src
cd ~/workspace/ros
catkin build
source install/setup.bash
cd ~/workspace/ros/src

source /opt/ros/$ROS_DISTRO/setup.bash
echo "Cloning pycram and dependencies"
if [ "$ROS_VERSION" = "1" ]; then
  vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram.rosinstall
else
  vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram-ros2.rosinstall
fi

echo "Setting up virtual environment"
python -m venv pycram/pycram-venv --system-site-packages
source pycram/pycram-venv/bin/activate
echo "Installing python dependencies"
pip install -U pip
pip install -U setuptools
pip install -r pycram/requirements.txt
echo "Checking for dependencies of other ros packages"
rosdep update && rosdep install --from-paths . -i -y
cd ..
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

printf 'Should IPython startup script be installed? [y/N]: '
read answer

if [ "$answer" != "${answer#[Yy]}" ] ;then
    if dpkg -s python3-ipython &>/dev/null; then
      echo 'Found IPython installation'
    else
      echo 'Installing IPython'
      sudo apt-get install python3-ipython
    fi
    cp src/pycram/script/ipython_helper/* ~/.ipython/profile_default/startup/
fi