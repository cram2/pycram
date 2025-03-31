=============
Compatability
=============

PyCRAM is compatibil and tested with **ROS1 Noetic/ Ubuntu20.04** and **ROS2 Jazzy/ Ubuntu24.04**.
ROS2 Humble/ Ubuntu22.04 is not usable since the creation of messages is more strict and some problems
may arise.

**Support for ROS1 Noetic/ Ubuntu20.04 will be dropped in July 25**


============
Installation
============

The setup of PyCRAM can be divided in four steps:
 * Install ROS
 * Installing Dependencies
 * Cloning the PyCRAM repo
 * Building your ROS workspace

This guide expects you to have a GitHub account with an SSH key (you can read about adding a new ssh key
`here <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>`_).

Installing ROS
==============

PyCRAM uses ROS for a variety of functionality, for this reason you need a working ROS installation on your machine.
For information on how to install ROS please refer to the official
documentation `here <http://wiki.ros.org/noetic/Installation/Ubuntu>`_ for ROS1 Noetic
or `here <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>`_ for ROS2 Jazzy.

Installing Dependencies
=======================

The dependencies you will need are:
    * vcstool
    * curl

These are available via the Ubuntu apt-repos and can be installed via the terminal:


.. code-block:: shell

    sudo apt-get install python3-vcstool curl


Using the install script
=========================
.. _install-script:

The easiest way to install PyCRAM is to use the install script. This script will install all necessary dependencies and
clone the PyCRAM repository into your ROS workspace. Furthermore, will it setup the Python environment for PyCRAM and
install the necessary Python packages.

Additionally the script can install the IPython startup scripts which come in handy when using PyCRAM in an IPython shell
for development.

To use the script you have to setup a ROS workspace before:

.. code-block:: shell

    mkdir -p ~/workspace/ros/src
    cd workspace/ros
    catkin build
    source install/setup.bash
    cd src

Now you can run the install script, *The scripts excepts that it is executed from /workspace/ros/src*:

.. code-block:: shell

    curl -s https://raw.githubusercontent.com/cram2/pycram/dev/scripts/install.sh | bash


PyCRAM on Ubuntu24.04 (ROS Jazzy)
===================================
.. _install-pycram-24:

Before installing PyCRAM you need to setup a ROS workspace into which PyCRAM can be cloned.

.. code-block:: shell

    mkdir -p ~/workspace/ros/src
    cd workspace/ros
    colcon build --symlink-install
    source install/setup.bash

If ``colcon build`` does not work this probably means that you did not source your ROS installation.
Source it by invoking:

.. code-block:: shell

    source /opt/ros/jazzy/setup.bash

Now you can install PyCRAM into your ROS workspace.

.. code-block:: shell

    cd ~/workspace/ros/src
    vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram-ros2.rosinstall --recursive
    cd ..
    colcon build --symlink-install
    source install/setup.bash
    echo "source ~/workspace/ros/install/setup.bash" >> ~/.bashrc

Afterwards continue with the steps under `Python Dependencies`_.

PyCRAM on Ubuntu 20.04 (ROS Noetic)
===================================
.. _install-pycram:

At first you need to install the catkin tools so we can use the ´´´colcon build´´´ command:

.. code-block:: shell

    apt install python3-catkin-tools

Now we can setup a ROS workspace into which PyCRAM can be cloned.

.. code-block:: shell

    mkdir -p ~/workspace/ros/src
    cd workspace/ros
    catkin build
    source devel/setup.bash

If ``catkin build`` does not work this probably means that you did not source your ROS installation.
Source it by invoking:

.. code-block:: shell

    source /opt/ros/noetic/setup.bash

Now you can install PyCRAM into your ROS workspace.

.. code-block:: shell

    cd ~/workspace/ros/src
    vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram.rosinstall --recursive
    rosdep update
    rosdep install --ignore-src --from-paths . -r
    cd ..
    catkin build
    source devel/setup.bash
    echo "source ~/workspace/ros/devel/setup.bash" >> ~/.bashrc

The cloning and setting up can take several minutes. After the command finishes you should see a number of repositories
in your ROS workspace.

The cloned repository contains the source code for PyCRAM as well as two short demos which demonstrate how to use it.

Building your ROS workspace
--------------------------

.. _build-ws:

Building and sourcing your ROS workspace using catkin compiles all ROS packages and manages the appending to the
respective PATH variables. This is necessary to be able to import PyCRAM via the Python import system and to find the
robot descriptions in the launch file.

If you have been following the tutorial steps until now you can skip this part.

You can build your ROS workspace with the following commands:

.. code-block:: shell

    cd ~/workspace/ros
    catkin build
    source devel/setup.bash


What the launch file does is start a ROS master, upload the robot URDF to the parameter server as well as starting the
IK solver.


Python Dependencies
===================

To install the Python dependencies we first need to setup a python-venv, we use the standard Python ``venv`` package
for this. To start you need to install the respective venv version of the package.

.. code-block:: shell

    apt install python<version>-venv

Now we need to navigate to the PyCRAM repository, create the Python venv and finally activate the venv.

.. code-block:: shell

    cd ~/workspace/ros/src/pycram
    python -m venv pycram-venv --system-site-packages
    source pycram-venv/bin/activate

Now for the final part we can install the Python dependencies of PyCRAM but before we do that we need to update the pip
and setuptools version.

.. code-block:: shell

    pip install -U pip
    pip install -U setuptools
    pip install -r requirements.txt

=======================
Verify the Installation
=======================



Building the documentation
==========================

The documentation uses jupyter-book as engine.
Building the documentation requires Python 3.9 to avoid dependency conflicts.
To install Python 3.9 on Ubuntu 20.04, use the following commands:

.. code-block:: shell

    sudo apt install python3.9

It is recommended to use a virtual environment to avoid conflicts with the system Python.

.. code-block:: shell

    apt-get install python3-virtualenv
    virtualenv -p python3.9 --system-site-packages build-doc
    source build-doc/bin/activate

Install the requirements in your python interpreter.

.. code-block:: shell

    cd ~/workspace/ros/src/pycram/doc
    pip install -r requirements.txt

Run pycram and build the docs.

.. code-block:: shell

    cd ~/workspace/ros
    roslaunch pycram ik_and_description.launch
    cd src/pycram/doc/source
    jupyter-book build .

Show the index.

.. code-block::

    firefox _build/html/index.html


==================================
Setup your Development Environment
==================================

If you installed PyCRAM and checked that the installation works you can continue with the next step of setting up
your development environment by following the link :ref:`here <setup env>`.
