.. _installation:

============
Installation
============

The setup of PyCRAM can be divided in four steps:
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
documentation `here <http://wiki.ros.org/noetic/Installation/Ubuntu>`_.

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

    cd ~/workspace/ros/src
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

    cd src/pycram
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

    cd ~/workspace/ros/src/pycram

Then install the Python packages in the requirements.txt file

.. code-block:: console

    sudo pip3 install -r requirements.txt
    sudo pip3 install -r src/neem_interface_python/requirements.txt


Building your ROS workspace
===========================

Building and sourcing your ROS workspace using catkin compiles all ROS packages and manages the appending to the
respective PATH variables. This is necessary to be able to import PyCRAM via the Python import system and to find the
robot descriptions in the launch file.

You can build your ROS workspace with the following commands:

.. code-block:: console

    cd ~/workspace/ros
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

Building the documentation
==========================

The documentation uses sphinx as engine.
Building sphinx based documentations requires pandoc
to be installed. Pandoc can be installed via the package manager of Ubuntu.

.. code-block:: console

    sudo apt install pandoc

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
ros workspace including pycram and source it as described in install-pycram_.

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

Using IPython as REPL
=====================

If you want to use a REPl with PyCRAM you can use IPython for that. IPython can be installed via
the Ubunutu package manager.

.. code-block:: console

    sudo apt install ipython3


Enable autoreload
-----------------

To use changes made in the Python file while the Repl is running you need to enable the iPython extension ``autoreload``.
This can be done using the iPython startup files, these are files which are always run if iPython is started.
The startup files are located in ``~/.ipython/profile_default/startup`` along with a README file which explains the usage
of the startup files. In this directory create a file called ``00-autoreload.ipy`` and enter the following code to the file.


.. code-block:: console

    %load_ext autoreload
    %autoreload 2

The first line loads the extension to iPython and the second line configures autoreload to reload all modules before the
code in the console is executed.


Run scripts
-----------

IPython allows to run Python files and enabled the access to created variables. This can be helpful
if you want to create a setup script which initializes things like the BulletWorld, Objects and imports
relevant modules.

To execute a Python script simply run ``run filename.py`` in the IPython console.

Here is an example how a setup script can look like.

.. code-block:: python

    from pycram.bullet_world import BulletWorld, Object
    from pycram.designators.action_designator import *
    from pycram.designators.motion_designator import *
    from pycram.designators.location_designator import *
    from pycram.designators.object_designator import *
    from pycram.process_module import simulated_robot

    world = BulletWorld()

    robot = Object("pr2", "robot", "pr2.urdf")
    kitchen = Object("kitchen", "environment", "kitchen.urdf")
    cereal = Object("cereal", "cereal", "breakfast_cereal.stl", position=[1.4, 1, 0.95])
