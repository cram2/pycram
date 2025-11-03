from pycram.language import SequentialPlanfrom pycram.language import SequentialPlanfrom pycram.language import SequentialPlanfrom pycram.language import SequentialPlanfrom semantic_digital_twin.adapters.urdf import URDFParser---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.3
  kernelspec:
    display_name: Python 3 (ipykernel)
    language: python
    name: python3
---


# Action Designator

This example will show the different kinds of Action Designators that are available. We will see how to create Action
Designators and what they do.

Action Designators are high-level descriptions of actions which the robot should execute.

Action Designators are created from an Action Designator Description, which describes the type of action as well as the
parameter for this action. Parameter are given as a list of possible parameters.
For example, if you want to describe the robot moving to a table you would need a
{meth}`~pycram.robot_plans.NavigateActionDescription` and a list of poses that are near the table or a 
LocationDesignator describing a pose near the table. The Action
Designator Description will then pick one of the poses and return a performable Action Designator which contains the
picked pose.

## Preface 
Action designator descriptions are able to handle a multitude of different inputs. In general, they are able to work with 
the argument directly or any iterable that generates the type of the argument. Iterables include a list of the arguments 
or another designator which generates the argument type. For example, a NavigateActionDescription takes as input a Pose 
now the possible input arguments for a NavigateActionDescription are: 

    * A Pose 
    * A list of Poses 
    * A Location Designator, since they are generating Poses  


## Navigate Action

We will start with a simple example of the {meth}`~pycram.robot_plans.NavigateAction`.

First, we need a BulletWorld with a robot.

All plans need a context in which they are performed, this context consists of the world as well as the robot that is to 
perform the plan. 

```python
import os

from semantic_digital_twin.adapters.urdf import  URDFParser
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.world import World
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.dataclasses import Context
from pycram.testing import setup_world

world = setup_world()
pr2_view = PR2.from_world(world)

context = Context(world, pr2_view)


```

To move the robot we need to create a description which will be resolved to the actual designator. The description of navigation
only needs a list of possible poses. In PyCRAM **every** designator needs to be part of a plan, the plan also manages the 
world in which the designator are executed as well as the robot which executes the plan. 

```python
from pycram.robot_plans import NavigateActionDescription
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan

pose = PoseStamped.from_list([1.3, 2, 0], [0, 0, 0, 1])

# This is the Designator Description
navigate_description = NavigateActionDescription(target_location=[pose])

# The plan containing the navigation designator
plan = SequentialPlan(context, navigate_description)
```

What we now did was: create the pose where we want to move the robot, create a description describing a navigation with
a list of possible poses (in this case the list contains only one pose) and create plan from the
description.

To execute the created plan just call perform on it. 

```python
from pycram.process_module import simulated_robot

with simulated_robot:
    plan.perform()
```

Every designator that is performed needs to be in an environment that specifies where to perform the designator either
on the real robot or the simulated one. This environment is called {meth}`~pycram.process_module.simulated_robot`  similar there is also
a {meth}`~pycram.process_module.real_robot` environment.

There are also decorators which do the same thing but for whole methods, they are called {meth}`~pycram.process_module.with_real_robot` 
and {meth}`~pycram.process_module.with_simulated_robot`.

## Move Torso

This action designator moves the torso up or down, specifically it sets the torso joint to a given value.

We start again by creating a description and resolving it to a designator. Afterwards, the designator is performed in
a {meth}`~pycram.process_module.simulated_robot` environment.

```python
from pycram.robot_plans import MoveTorsoActionDescription
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import TorsoState
from pycram.language import SequentialPlan

torso_pose = TorsoState.HIGH

torso_desig = MoveTorsoActionDescription([torso_pose])

plan = SequentialPlan(context, torso_desig)

with simulated_robot:
    plan.perform()
```

## Set Gripper

As the name implies, this action designator is used to open or close the gripper.

The procedure is similar to the last time, but this time we will shorten it a bit.

```python
from pycram.robot_plans import SetGripperActionDescription
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import GripperState, Arms
from pycram.language import SequentialPlan

gripper = Arms.RIGHT
motion = GripperState.OPEN

with simulated_robot:
    SequentialPlan(context, SetGripperActionDescription(gripper=gripper, motion=[motion])).perform()
```

## Park Arms

Park arms is used to move one or both arms into the default parking position.

```python
from pycram.robot_plans import ParkArmsActionDescription
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan

with simulated_robot:
    SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH)).perform()
```

## Pick Up and Place

Since these two are dependent on each other, meaning you can only place something when you picked it up beforehand, they
will be shown together.

These action designators use object designators, which will not be further explained in this tutorial so please check
the example on object designators for more details.

To start we need an environment in which we can pick up and place things as well as an object to pick up.

```python
from pycram.robot_plans import PickUpActionDescription, PlaceActionDescription, ParkArmsActionDescription, MoveTorsoActionDescription, NavigateActionDescription
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import Arms, ApproachDirection, TorsoState, VerticalAlignment
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.grasp import GraspDescription
from pycram.language import SequentialPlan

arm = Arms.RIGHT

with simulated_robot:
    SequentialPlan(context,
        ParkArmsActionDescription(Arms.BOTH),
    
        MoveTorsoActionDescription([TorsoState.HIGH]),
    
        NavigateActionDescription([PoseStamped.from_list([1.8, 2.2, 0.0],
                                                         [0.0, 0.0, 0., 1])]),
    
        PickUpActionDescription(object_designator=world.get_body_by_name("milk.stl"),
                                arm=[arm],
                                grasp_description= GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)),
    
        PlaceActionDescription(object_designator=world.get_body_by_name("milk.stl"),
                               target_location=[PoseStamped.from_list([2.4, 1.8, 1],
                                                                      [0, 0, 0, 1], world.root)],
                               arm=arm)).perform()
```

## Look At

Look at lets the robot look at a specific point, for example if it should look at an object for detecting.


```python
from pycram.robot_plans import LookAtActionDescription
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import PoseStamped

target_location = PoseStamped.from_list([3, 2, 1], [0, 0, 0, 1], world.root)
with simulated_robot:
    SequentialPlan(context, LookAtActionDescription(target=[target_location])).perform()
```

## Detect

Detect is used to detect objects in the field of vision (FOV) of the robot. We will use the milk used in the pick
up/place example, if you didn't execute that example you can spawn the milk with the following cell. The detect
designator will return a resolved instance of an ObjectDesignatorDescription.


```python
# from pycram.robot_plans import DetectActionDescription, LookAtActionDescription, ParkArmsActionDescription, NavigateActionDescription
# from pycram.designators.object_designator import BelieveObject
# from pycram.datastructures.enums import Arms
# from pycram.process_module import simulated_robot
# from pycram.datastructures.pose import PoseStamped
# from pycram.datastructures.enums import DetectionTechnique
# 
# milk_desig = BelieveObject(names=["milk"])
# 
# with simulated_robot:
#     ParkArmsActionDescription([Arms.BOTH]).resolve().perform()
# 
#     NavigateActionDescription([PoseStamped.from_list([1.7, 2, 0], [0, 0, 0, 1])]).resolve().perform()
# 
#     LookAtActionDescription(target=milk_desig.resolve().pose).resolve().perform()
# 
#     obj_desig = DetectActionDescription(DetectionTechnique.ALL,
#                                         object_designator=milk_desig).resolve().perform()
# 
#     print(obj_desig)
```

## Transporting

Transporting can transport an object from its current position to another target position. It is similar to the Pick and
Place plan used in the Pick-up and Place example. Since we need an Object which we can transport we spawn a milk, you
don't need to do this if you already have spawned it in a previous example.


```python
from pycram.robot_plans import *
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.enums import Arms, TorsoState


description = TransportActionDescription(world.get_body_by_name("milk.stl"),
                                         [PoseStamped.from_list([3, 2.2, 0.95],
                                                                [0.0, 0.0, 1.0, 0.0], world.root)],
                                         [Arms.LEFT])
with simulated_robot:
    SequentialPlan(context,
        MoveTorsoActionDescription([TorsoState.HIGH]),
        description).perform()
```

## Opening

Opening allows the robot to open a drawer, the drawer is identified by an ObjectPart designator which describes the
handle of the drawer that should be grasped.

For the moment this designator works only in the apartment environment, therefore we remove the kitchen and spawn the
apartment.

```python
from pycram.robot_plans import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import Arms, TorsoState
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import PoseStamped


with simulated_robot:
    SequentialPlan(context,
        MoveTorsoActionDescription([TorsoState.HIGH]),
        ParkArmsActionDescription([Arms.BOTH]),
        NavigateActionDescription([PoseStamped.from_list([1.7474915981292725, 2.6873629093170166, 0.0],
                                               [-0.0, 0.0, 0.5253598267689507, -0.850880163370435])]),
        OpenActionDescription(world.get_body_by_name("handle_cab10_t"), [Arms.RIGHT])).perform()
```

## Closing

Closing lets the robot close an open drawer, like opening the drawer is identified by an ObjectPart designator
describing the handle to be grasped.

This action designator only works in the apartment environment for the moment, therefore we remove the kitchen and spawn
the apartment. Additionally, we open the drawer such that we can close it with the action designator.

```python
from pycram.robot_plans import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import Arms
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import PoseStamped

with simulated_robot:
    SequentialPlan(context,
        MoveTorsoActionDescription([TorsoState.HIGH]),
        ParkArmsActionDescription([Arms.BOTH]),
        NavigateActionDescription([PoseStamped.from_list([1.7474915981292725, 2.6873629093170166, 0.0],
                                               [-0.0, 0.0, 0.5253598267689507, -0.850880163370435])]),
        CloseActionDescription(world.get_body_by_name("handle_cab10_t"), [Arms.RIGHT])).perform()
```
