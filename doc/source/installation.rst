==================
Installation Guide
==================


=============
Compatability
=============

PyCRAM is compatible and tested with **ROS2 Jazzy/ Ubuntu24.04 and Python 3.12**.

PyCRAM and ROS
--------------
While PyCRAM works on its own and is not dependent on ROS to work some of its functionallity requires a ROS environment.
This includes:

* Loading URDFs
* Using the visualization with RVIZ2
* Synchronizing between different processes

**This is especially required to run the examples**

Installation with Pip
=====================
The easiest way to install PyCRAM is via Pip. For this first setup a virtual environment using the following command:

.. code-block:: python

    python3 -m venv pycram-venv --system-sitepackages

Then just activate the environment and install PyCRAM

.. code-block:: python

    source pycram-venv/bin/activate
    pip install pycram-robotics


Installation for Development
============================

For development it is recommended to use the virtualenvwrapper package to setup the virtual environment.
To install the virtualenvwrapper just execute the following command:

.. code-block:: bash
    sudo apt install virtualenvwrapper

Set virtualenvwrapper's `WORKON_HOME` env variable, of which the default value is `~/.virtualenvs`

.. code-block:: shell

    echo "export WORKON_HOME=~/envs" >> ~/.bashrc
    mkdir -p $WORKON_HOME

Activate virtualenvwrapper at terminal start

.. code-block:: shell

    echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
    source ~/.bashrc

Create a virtual env based on the workspaces libraries (see build-ws_) and add the `--system-site-packages` to get them properly. The env will be registered in `$WORKON_HOME`.

.. code-block:: shell

    mkvirtualenv pycram --system-site-packages
    ls $WORKON_HOME

Now you can activate the venv from anywhere using the command:

.. code-block:: shell

    workon pycram

Afterwards clone the PyCRAM github repo and install it into the venv.

.. code-block:: shell

    git clone git@github.com:cram2/pycram.git
    pip install -e pycram

Remeber that the git clone command requires you to have a valid ssh-key in your GitHub account (you can read about adding a new ssh key
`here <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>`_).

Using PyCharm
-------------
We recommend using PyCharm for development. PyCharm might have trouble finding ROS dependencies since they are not added
to the PYTHON_PATH by default. To solve this problem start PyCharm from a terminal in which ROS is sourced.

.. code-block:: shell

    source /opt/ros/jayyz/setup.bash
    pycharm

Instead of "pycharm" the command could also be "pycharm-professional" that depends on your installation. Just use <TAB>
to autocomplete.



Install ROS Dependencies
========================
To execute the examples or tests of PyCRAM you need to setup a ROS environment and the dependency packages that are
needed by ROS.
For your convenience there is a script to install ROS and setup the dependencies for PyCRAM

.. code-block:: shell

    curl -s https://raw.githubusercontent.com/cram2/pycram/dev/scripts/setup_ros.sh | bash


========
Appendix
========

The instructions here are not needed to work with PyCRAM and are just additional info.


Setup your Development Environment
==================================

If you installed PyCRAM and checked that the installation works you can continue with the next step of setting up
your development environment by following the link :ref:`here <setup_env>`.


Manual Setup
------------
If you already have a ROS setup you are probably already aware how to install and build ROS packages. In that case
you need install and build these two ROS packages:

* https://github.com/code-iai/iai_maps
* https://github.com/code-iai/iai_pr2


**Make sure to select the correct branch for ros-jazzy**

Building the documentation
==========================

The documentation uses jupyter-book as engine.
Building the documentation requires Python 3.9 or higher to avoid dependency conflicts.

Source the venv that contains PyCRAM

.. code-block:: shell

    source pycram-venv/bin/activate

Install the requirements in your python interpreter.

.. code-block:: shell

    cd <path-to-repo>/doc
    pip install -r requirements.txt

Run pycram and build the docs.

.. code-block:: shell

    cd <path-to-repo>/doc/source
    jupyter-book build .

Show the index.

.. code-block::

    firefox _build/html/index.html


