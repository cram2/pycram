============================
Adding a new robot to PyCRAM
============================

To add a new robot to PyCRAM, you need two things:
    * A Robot description
    * Process Modules to control the robot

--------------------------------
Robot Description
--------------------------------
The robot description defines certain semantic properties of the robot that can not be extracted from the robot's URDF automatically.
This includes kinematic chains which the robot can move (like the arms), descriptions of the end-effectors as well as
descriptions of the cameras mounted on the robot.

An overview of the different components of the robot description as well as how these are created can be found in the
following example:

:ref:`Robot Description example<robot_description_header>`




--------------------------------
Process Modules
--------------------------------
Process Modules are the components that actually control the robot. They are responsible for executing actions on the robot
and are the only component of PyCRAM that interacts with the robot directly.

If you want to use the robot in simulation then you can use the provided default process modules which will suffice to
control the robot in simulation. However, should the provided process modules not be sufficient for your use case, you can
implement them yourself. For examples how the process modules are designed please have a look at the already implemented
process modules in :mod:`pycram.process_modules`.