---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.2
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

# Giskard interface in PyCRAM

This notebook should provide you with an example on how to use the Giskard interface. This includes how to use the
interface, how it actually works, and how to extend it.

We start by installing and launching Giskard. For the installation please follow the
instructions [here](https://github.com/SemRoCo/giskardpy).
After you finish the installation you should be able to launch Giskard by calling:

```
roslaunch giskardpy giskardpy_pr2_standalone.launch
```

This way you can launch Giskard for any robot that is supported:

```
roslaunch giskardpy giskardpy_hsr_standalone.launch
```

"Standalone" means that Giskard only uses a simulated robot and does not look for a real robot. If you want to use
Giskard with a real robot you have to switch out "standalone" with "iai", e.g:

```
roslaunch giskardpy giskardpy_hsr_iai.launch
```

To see what Giskard is doing you can start RViz, there should already be a MarkerArray when starting otherwise you have
to add this manually.

## How to use the Giskard interface

Everything related to the Giskard interface is located in {class}`pycram.external_interfaces.giskard`.
The content of the file can be roughly divided into three parts:
1. Methods to manage the belief states between PyCRAM and Giskard
2. Motion goals that should be sent to Giskard for execution
3. Helper methods to construct ROS messages

The most useful methods are the ones for sending and executing Motion goals. These are the ones we will mostly look at.

We will now start by setting up PyCRAM and then try to send some simple motion goals.

```python
from pycram.bullet_world import BulletWorld, Object
from pycram.enums import ObjectType

world = BulletWorld()
pr2 = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
```

When you are working on the real robot you also need to initialize the RobotStateUpdater, this module updates the robot
in the BulletWorld with the pose and joint state of the real robot.

You might need to change to topic names to fit the topic names as published by your robot.

```python
from pycram.ros_utils.robot_state_updater import RobotStateUpdater

r = RobotStateUpdater("/tf", "/joint_states")
```

Now we have a PyCRAM belief state set up, belief state in this case just refers to the BulletWorld since the BulletWorld
represents what we believe the world to look like.

The next step will be to send a simple motion goal. The motion goal we will be sending is moving the torso up. For this
we just need to move one joint, so we can use the ```achive_joint_goal```. This method takes a dictionary with the
joints that should be moved and the target value for the joints.

Look at RViz to see the robot move, since we call Giskard for movement the robot in the BulletWorld will not move.

```python
from pycram.external_interfaces import giskard

giskard.achieve_joint_goal({"torso_lift_joint": 0.28})
```

For Giskard everything is connected by joints (this is called
a [World Tree](https://github.com/SemRoCo/giskardpy/wiki/World-Tree) by Giskard) therefore we can move the robot's base
by using motion goals between the map origin and the robot base. (e.g. by sending a "base_link" goal in the "map"
frame).

In the example below we use a cartesian goal, meaning we give Giskard a goal pose, a root link and a tip link and
Giskard tries to move all joints between root link and tip link such that the tip link is at the goal pose.

This sort of movement is fine for short distances, but keep in mind that Giskard has no collision avoidance for longer
journeys. So using MoveBase for longer distances is a better idea.

```python
from pycram.external_interfaces import giskard
from pycram.pose import Pose

giskard.achieve_cartesian_goal(Pose([1, 0, 0]), "base_link", "map")
```

Now for the last example: we will move the gripper using full body motion control.

We will again use the cartesian goal, but now between "map" and "r_gripper_tool_frame" frames. This will not only move
the robot (because the kinematic chain between "map" and "base_link" as used in the previous example is also part of
this chain) but also move the arm of the robot such that it reaches the goal pose.

```python
from pycram.external_interfaces import giskard
from pycram.pose import Pose

giskard.achieve_cartesian_goal(Pose([1, 0.5, 0.7]), "r_gripper_tool_frame", "map")
```

That concludes this example you can now close the BulletWorld by using the "exit" method.

```python
world.exit()
```

## How the Giskard interface works

The PyCRAM interface to Giskard mostly relies on the Python interface that Giskard already
provides ([tutorial](https://github.com/SemRoCo/giskardpy/wiki/Python-Interface) and
the [source code](https://github.com/SemRoCo/giskardpy/blob/master/src/giskardpy/python_interface.py)). This interface
provides methods to achieve motion goals and load things into the Giskard believe state.

What PyCRAM does with this, is: Synchronize the belief state of Giskard with the one of PyCRAM by loading the
environment URDF in Giskard, this is done before any motion goal is sent. Furthermore, the motion goals are wrapped in
methods that use PyCRAM data types.

You can also set collisions between different groups of links. By default, Giskard avoids all collisions but for things
like grasping an object you want to allow collisions of the gripper. The interface also supports the following collision
modes:
* avoid_all_collisions
* allow_self_collision
* allow_gripper_collision
The collision mode can be set by calling the respective method, after calling the method the collision mode is valid for
the next motion goal. Afterwards, it defaults back to avoid_all_collisions.

There is a ```init_giskard_interface``` method which can be used as a decorator. This decorator should be used on all
methods that access the giskard_wrapper, since it assures that the interface is working and checks if Giskard died or
the imports for the giskard_msgs failed.

## Extend the Giskard interface

At the moment the PyCRAM Giskard interface is mostly a wrapper around the Python interface provided by Giskard. If you
want to extend the interface there are two ways:

* Wrap more motion goals which are provided by the Python interface
* Design new Higher-Level motion goals by combining the motion goals already provided
