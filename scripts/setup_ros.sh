#!/bin/bash

echo "Setting up ROS"

echo "Updating Locals"

locale  # check for UTF-8

sudo apt update && sudo apt install locales

sudo locale-gen en_US en_US.UTF-8

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

locale  # verify settings


echo "Adding additional Repos"
sudo apt install software-properties-common

sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install ros-dev-tools -y

echo "Installing ROS"

sudo apt install ros-jazzy-desktop -y


echo "Setting up workspace structure"
mkdir -p ~/workspace/ros/src
cd ~/workspace/ros
cd ~/workspace/ros/src

echo "Source ROS installation"
source /opt/ros/$ROS_DISTRO/setup.bash

echo "Installing xacro and ROS dependencies"
sudo apt install ros-"${ROS_DISTRO}"-xacro -y
echo "Cloning pycram and dependencies"
vcs import --input https://raw.githubusercontent.com/cram2/pycram/main/rosinstall/pycram-ros2-dependencies.rosinstall

cd ~/workspace/ros

colcon build --symlink-install
source install/setup.bash
echo "source ~/workspace/ros/install/setup.bash" >> ~/.bashrc

echo "Successfully installed ROS "