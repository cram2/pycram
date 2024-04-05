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

This guide expects you to have a GitHub account with an SSH key (you can read about adding a new ssh key
`here <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>`_).

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


.. code-block:: shell

    sudo apt-get install python3-pip python3-vcstool

PyCRAM on Ubuntu 20.04 (ROS Noetic)
===================================
.. _install-pycram:

Before installing PyCRAM you need to setup a ROS workspace into which PyCRAM can be cloned.

.. code-block:: shell

    mkdir -p ~/workspace/ros/src
    cd workspace/ros
    catkin_make
    source devel/setup.bash

If ``catkin_make`` does not work this probably means that you did not source your ROS installation.
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
    catkin_make
    source devel/setup.bash
    echo "~/workspace/ros/devel/setup.bash" >> ~/.bashrc

The cloning and setting up can take several minutes. After the command finishes you should see a number of repositories
in your ROS workspace.

Now the last thing that needs to be done is clone the submodules of the PyCRAM repo, this is done via the following
commands.

.. code-block:: shell

    cd src/pycram
    git submodule init
    git submodule update

The cloned repository contains the source code for PyCRAM as well as two short demos which demonstrate how to use it.

Python Dependencies
===================

To install the Python dependencies Pip is used. To install Pip type the following command into a terminal.

.. code-block:: shell

    sudo apt-get install python3-pip

Now the actual Python packages can be installed, these are summarized in the requirements.txt in the PyCRAM repo.
For this first navigate to your PyCRAM repo.

.. code-block:: shell

    cd ~/workspace/ros/src/pycram

Then install the Python packages in the requirements.txt file

.. code-block:: shell

    sudo pip3 install -r requirements.txt
    sudo pip3 install -r src/neem_interface_python/requirements.txt


Building your ROS workspace
===========================
.. _build-ws:

Building and sourcing your ROS workspace using catkin compiles all ROS packages and manages the appending to the
respective PATH variables. This is necessary to be able to import PyCRAM via the Python import system and to find the
robot descriptions in the launch file.

If you have been following the tutorial steps until now you can skip this part. 

You can build your ROS workspace with the following commands:

.. code-block:: shell

    cd ~/workspace/ros
    catkin_make
    source devel/setup.bash

Using PyCRAM
============

To start using PyCRAM you first need to launch the ROS launch file. This launchfile is located in the directory "launch"
and is named "ik_and_description.launch".

The launchfile can be started with the following command:

.. code-block:: shell

    roslaunch pycram ik_and_description.launch


What the launch file does is start a ROS master, upload the robot URDF to the parameter server as well as starting the
IK solver.

Building the documentation
==========================

The documentation uses sphinx as engine.
Building sphinx based documentations requires pandoc
to be installed. Pandoc can be installed via the package manager of Ubuntu.

.. code-block:: shell

    sudo apt install pandoc

After installing pandoc, install sphinx on your device.

.. code-block:: shell

    sudo apt install python3-sphinx


Install the requirements in your python interpreter.

.. code-block:: shell

    cd ~/workspace/ros/src/pycram/doc
    pip install -r requirements.txt

Run pycram and build the docs.

.. code-block:: shell

    cd ~/workspace/ros
    roslaunch pycram ik_and_description.launch
    cd src/pycram/doc
    make html

Show the index.

.. code-block::

    firefox build/html/index.html



Setting up PyCRAM with PyCharm
==============================

Setting up PyCharm with packages that rely on rospy is non trivial. Follow this guide to get correct syntax highlighting for the PyCRAM project. 

Install PyCharm Professional
----------------------------

First, `install PyCharm Professional <https://www.jetbrains.com/help/pycharm/installation-guide.html#standalone>`_.

Create a JetBrains account and verify it for educational purpose. Normally, a school email address would suffice, otherwise you would have to upload your student/employee id card. The verification process typically takes 1~2-week time, so until then please use Trial version.
Once your account is verified, you can unlock the PyCharm Professional features in PyCharm.

The next step will set up the virtual Python environment, so it can be used as a project interpreter in PyCharm. 


Set up virtualenv
-----------------
.. _virtualenv:

The virtualenvwrapper allows to manage virtual Python environments, where additional packages can be installed without the risk of breaking the system-wide Python configuration. Install `virtualenvwrapper <https://virtualenvwrapper.readthedocs.io/en/latest/>`_ via pip and set it up.

.. code-block:: shell

    sudo pip3 install virtualenvwrapper


(Optional but recommended) Set virtualenvwrapper's `WORKON_HOME` env variable, of which the default value is `~/.virtualenvs`

.. code-block:: shell

    echo "export WORKON_HOME=~/envs" >> ~/.bashrc
    mkdir -p $WORKON_HOME

Activate virtualenvwrapper at terminal start

.. code-block:: shell

    echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
    source ~/.bashrc

Create a virtual env based on the workspaces libraries (see build-ws_) and add the `--system-site-packages` to get them properly. The env will be registered in `$WORKON_HOME`.

.. code-block:: shell

    source ~/workspace/ros/devel/setup.bash
    mkvirtualenv pycram --system-site-packages
    ls $WORKON_HOME


Check if the ROS libraries are available in the virtual env.

.. code-block:: shell

    workon pycram
    python -c "import rospy"

If it complains about `python`, install the following, to set `python` to Python 3 by default.

.. code-block:: shell

    sudo apt install python-is-python3  

If it finds `python` but complains about missing packages, make sure that the workspace is sourced before creating the virtual env. Also remember to create the virtual env with the `--system-site-packages` flag.

If this returns no errors, you can be sure that rospy is usable in your virtual environment. Next you have to build the
ros workspace including pycram and source it as described in build-ws_.

Configure PyCharm
-----------------

Always start PyCharm from the terminal via

.. code-block:: shell

    pycharm-professional

or

.. code-block:: shell

    ~/pycharm/bin/pycharm.sh


Select **File | Open** and select the root folder of the PyCRAM package.
Next go to **File | Settings | Project: pycram | Python Interpreter** and set up your virtual environment with rospy and
the sourced workspace available as the python interpreter.

Finally, go to  **File | Settings | Project: pycram | Project Structure** and mark the src folder as Sources, the test
folder as Tests and the resources as Resources.

To verify that it works, you can execute any Testcase.

**Useful tips**

- `Keyboard shortcuts <https://www.jetbrains.com/help/pycharm/mastering-keyboard-shortcuts.html>`_
    - `Keymap <https://www.jetbrains.com/help/pycharm/mastering-keyboard-shortcuts.html#ws_print_keymap>`_

- `Python interpreter <https://www.jetbrains.com/help/pycharm/configuring-python-interpreter.html>`_
    - `Python virtual environment <https://www.jetbrains.com/help/pycharm/creating-virtual-environment.html>`_
- `Python packages <https://www.jetbrains.com/help/pycharm/installing-uninstalling-and-upgrading-packages.html>`_
- `Python console <https://www.jetbrains.com/help/pycharm/using-consoles.html>`_

- **View | Active Editor | Soft-wrap**: wrap text inside the editor view

- **View | Tool Windows | Structure**: display structure window for easy content navigation

- **F12**: Open terminal

- **Double Shift**: Quick file search

- **Ctrl F/R**: Find/Replace text in current file

- **Ctrl Shift F/R**: Find/Replace text in the whole project, module, directory, scope

- **Settings | Editor | Inspections | Code is compatible with specific Python versions**: Enable/Disable Python version-specific warnings

Using IPython as REPL
=====================

If you want to use a REPl with PyCRAM you can use IPython for that. IPython can be installed via
the Ubunutu package manager.

.. code-block:: shell

    sudo apt install ipython3


Enable autoreload
-----------------

To use changes made in the Python file while the Repl is running you need to enable the iPython extension ``autoreload``.
This can be done using the iPython startup files, these are files which are always run if iPython is started.
First run ``ipython profile create`` to create a `default profile <https://ipython.readthedocs.io/en/stable/config/intro.html>`_.
Then you will find the startup files located in ``~/.ipython/profile_default/startup`` along with a README file which explains the usage
of the startup files. In this directory create a file called ``00-autoreload.ipy`` and enter the following code to the file.


.. code-block:: shell

    %load_ext autoreload
    %autoreload 2

The first line loads the extension to iPython and the second line configures autoreload to reload all modules before the
code in the shell is executed.


Run scripts
-----------

IPython allows to run Python files and enables the access to created variables. This can be helpful
if you want to create a setup script which initializes things like the BulletWorld, Objects and imports
relevant modules.

To execute a Python script simply run ``run filename.py`` in the IPython shell.

Here is an example how a setup script can look like.

.. code-block:: python

    from pycram.bullet_world import BulletWorld, Object
    from pycram.designators.action_designator import *
    from pycram.designators.motion_designator import *
    from pycram.designators.location_designator import *
    from pycram.designators.object_designator import *
    from pycram.process_module import simulated_robot
    from pycram.pose import Pose
    from pycram.enums import ObjectType

    world = BulletWorld()

    robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
    kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
    cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([1.4, 1, 0.95]))
