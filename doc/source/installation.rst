.. _installation:

============
Installation
============

The setup of PyCRAM can be differentiated in four steps:
 * Install ROS
 * Installing Dependencies
 * Cloning the PyCRAM repo
 * Building your ROS workspace

All dependencies are available via PyPi.

PyCRAM is developed and tested currently with Python3.8, Ubuntu 20.04 and ROS Noetic.

Installing ROS
==============

PyCRAM uses ROS for a variety of functionality, for this reason you need a working ROS installation on your machine.
For information on how to install ROS please referee to the official
documentation `here <http://wiki.ros.org/ROS/Installation>`_.

Installing Dependencies
=======================

The dependencies you will need are:
    * Pip
    * vcstool
These are available via the Ubuntu apt-repos and can be installed via the terminal:


.. code-block:: console

    sudo apt-get install python3-pip python3-vcstool

PyCRAM on Ubuntu 20.04 (ROS Noetic)
===================================
.. _install-pycram:

Before installing PyCRAM you need to setup a ROS workspace into which PyCRAM can be cloned.

.. code-block:: console

    mkdir -p ~/workspace/ros/src
    cd workspace/ros
    catkin_make
    source devel/setup.bash

If ``catkin_make`` does not work this probably means that you did not source your ROS installation.
Source it by invoking:

.. code-block:: console

    source /opt/ros/noetic/setup.bash


Now you can install PyCRAM into your ROS workspace.

.. code-block:: console

    roscd
    vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram.rosinstall --recursive
    rosdep update
    rosdep install --ignore-src --from-paths . -r
    cd ..
    catkin_make

The cloning and setting up can take several minutes. After the command finishes you should see a number of repositories
in your ROS workspace.

Now the last thing that needs to be done is clone the submodules of the PyCRAM repo, this is done via the following
commands.

.. code-block:: console

    cd pycram
    git submodule init
    git submodule update

The cloned repository contains the source code for PyCRAM as well as two short demos which demonstrate how to use it.

Python Dependencies
===================

To install the Python dependencies Pip is used. To install Pip type the following command into a terminal.

.. code-block:: console

    sudo apt-get install python3-pip

Now the actual Python packages can be installed, these are summarized in the requirements.txt in the PyCRAM repo.
For this first navigate to your PyCRAM repo.

.. code-block:: console

    cd <path-to-your-pycram-repo>

Then install the Python packages in the requirements.txt file

.. code-block:: console

    sudo pip install -r requirements.txt
    sudo pip install -r src/neem_interface_python/requirements.txt


Building your ROS workspace
===========================

Building and sourcing your ROS workspace using catkin compiles all ROS packages and manages the appending to the
respective PATH variables. This is necessary to be able to import PyCRAM via the Python import system and to find the
robot descriptions in the launch file.

You can build your ROS workspace with the following commands:

.. code-block:: console

    cd <Path to your ROS workspace>
    catkin_make
    source devel/local_setup.bash

Using PyCRAM
============

To start using PyCRAM you first need to launch the ROS launch file. This launchfile is located in the directory "launch"
and is named "ik_and_description.launch".

The launchfile can be started with the following command:

.. code-block:: console

    roslaunch pycram ik_and_description.launch


What the launch file does is start a ROS master, upload the robot URDF to the parameter server as well as starting the
IK solver.

Disclaimer
----------

At the moment you also need a knowrob node running for PyCRAM to start. This is because while importing packages some
will look for the rosprolog services. You don't need a belief state, it only requires the rosprolog services to be
reachable.

PyCRAM on Ubuntu 18.04 (ROS Melodic)
====================================

To be able to use PyCRAM on Ubuntu 18.04 you need a few extra steps because ROS melodic doesn't fully support Python 3.
The first thing you need to do is install Python3 pip.

.. code-block:: console

    apt-get install python3-pip

Next you need to install the Python dependencies using pip

.. code-block:: console

    pip3 install rospkg empy numpy

So far you should be able to import rospy in Python and use all features but for PyCRAM to function you also need the ROS tf package which is unfortunately not available in Python 3. To be able to use the tf package we will compile it our self for Python 3.

Build Tf for Python 3
=====================

Firstly you need to clone the geometry and geometry2 repos into your ROS workspace and select the melodic branches.

.. code-block:: console

    roscd
    git clone git@github.com:ros/geometry.git
    cd geometry
    git checkout melodic-devel
    cd ..
    git clone git@github.com:ros/geometry2.git
    cd geometry2
    git checkout melodic-devel

Now all you need to do is source ROS and build your workspace using for Python 3.

.. code-block:: console

    source /opt/ros/melodic/setup.bash
    cd ur_ros_ws/
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3.6

This should build the Tf package for Python 3. Now you can source your workspace and use the Tf package.

In order to use ROS and the Python3 Tf package you have to pay attention to a little thing when sourcing ROS. It is not enough to just source your workspace, you need to source the '/opt/ros/melodic/setup.bash' before hand to be able to use roslaunch.

The easiest way is to add the two sourcing commands to your .bashrc like so.

.. code-block:: console

    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc


Build PyKDL for Python 3
========================

Now you need to build PyKDL as well as kdl_parser_py for Python 3.  This is done in two distinctive steps, first build orocos_kdl using cmake outside of your catkin workspace and then building your catkin workspace containing PyKDL and kdl_parser_py for Python 3.
First clone the orocos kinematics dynamicas Repo, outside of your catkin workspace.

.. code-block:: console

    git clone git@github.com:orocos/orocos_kinematics_dynamics.git

Now build the orocos_kdl library, by pasting the following commands in a terminal.

.. code-block:: console

    cd orocos_kinematics_dynamics/orocos_kdl
    mkdir build
    cd build
    cmake ..
    make
    sudo make install


Now all you have to do is copy the PyKDL package from the orocos kinematics dynamics folder to your catkin workspace, clone the kdl_parser_py and build.

.. code-block:: console

    cd orocos_kinematics_dynamics
    git submodule update --init
    cp python_orocos_kdl path/to/your/catkin/workspace
    cd <path to yourt catkin workspace>/python_orocos_kdl
    cd ../

This will copy python orocos kdl into your catkin workspace and clone the submodules contained in the repo. Now we have to clone the kdl_parser_py and build.

.. code-block:: console

    git clone git@github.com:ros/kdl_parser.git
    cd ../..
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3.6


Building the documentation
==========================

The documentation uses sphinx as engine.
Building sphinx based documentations requires `pandoc <https://pandoc.org/installing.html>`_
to be installed.
After installing pandoc, install sphinx on your device.

.. code-block:: console

    sudo apt install python3-sphinx

Install the requirements in your python interpreter.

.. code-block:: console

    pip install -r requirements.txt

Run pycram and build the docs.

.. code-block:: console

    roslaunch pycram ik_and_description.launch
    make html

Show the index.

.. code-block::

    firefox build/html/index.html


Setting up PyCRAM with PyCharm
==============================

Setting up PyCharm with packages that rely on rospy is non trivial. Follow this guide to get correct syntax highlighting
for the PyCRAM project.

First, `install PyCharm Professional <https://www.jetbrains.com/help/pycharm/installation-guide.html#standalone>`_.

Next, if you have virtual environments that you want to use, you need to make sure that they have rospy available.
If you create a new environment, make sure to include  `--system-site-packages` in your creation command.
You can check by activating your environment and calling the import

.. code-block:: console

    workon your_env
    python -c "import rospy"

If this returns no errors, you can be sure that rospy is usable in your virtual environment. Next you have to build the
ros workspace including pycram and source it as described in :ref:`install-pycram`.

After that you have to start PyCharm from the terminal via

.. code-block:: console

    pycharm-professional

or

.. code-block:: console

    ~/pycharm/bin/pycharm.sh

Select **File | Open** and select the root folder of the PyCRAM package.
Next go to **File | Settings | Project: pycram | Python Interpreter** and set up your virtual environment with rospy and
the sourced workspace available as the python interpreter.

Finally, go to  **File | Settings | Project: pycram | Project Structure** and mark the src folder as Sources, the test
folder as Tests and the resources as Resources.

To verify that it works, you can execute any Testcase.