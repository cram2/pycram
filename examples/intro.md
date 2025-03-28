---
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

# PyCRAM Introduction

```python
import pycram
```

# Bullet World

The BulletWorld is the internal simulation of PyCRAM. You can simulate different actions and reason about the outcome of
different actions.

It is possible to spawn objects and robots into the BulletWorld, these objects can come from URDF, OBJ or STL files.

A BulletWorld can be created by simply creating an object of the BulletWorld class.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import PoseStamped
from pycrap.ontologies import Milk, Cereal, Robot, Kitchen

world = BulletWorld(mode=WorldMode.DIRECT)

milk = Object("milk", Milk, "milk.stl")
pr2 = Object("pr2", Robot, "pr2.urdf")
cereal = Object("cereal", Cereal, "breakfast_cereal.stl", pose=PoseSteamped.from_list([1.4, 1, 0.95]))
```

The BulletWorld allows to render images from arbitrary positions. In the following example we render images with the
camera at the position [0.3, 0, 1] and pointing towards [1, 0, 1], so we are looking upwards along the x-axis.

The renderer returns 3 different kinds of images which are also shown on the left side of the BulletWorld window. (If
these winodws are missing, click the BulletWorld window to focus it, and press "g") These images are:

* An RGB image which shows everything like it is rendered in the BulletWorld window, just from another perspective.
* A depth image which consists of distance values from the camera towards the objects in the field of view.
* A segmentation mask image which segments the image into the different objects displayed. The segmentation is done by
  assigning every pixel the unique id of the object that is displayed there.

```python
world.get_images_for_target(Pose([1, 0, 1], [0, 0, 0, 1]), Pose([0.3, 0, 1], [0, 0, 0, 1]))
```

## Objects

Everything that is located inside the BulletWorld is an Object.
Objects can be created from URDF, OBJ or STL files. Since everything is of type Object a robot might share the same
methods as a milk (with some limitations).

Signature:
Object:

* Name
* Type
* Filename or Filepath

Optional:

* Position
* Orientation
* World
* Color
* Ignore Cached Files

Objects provide methods to change the position and rotation, change the color, attach other objects, set the state of
joints if the objects has any or get the position and orientation of a link.

These methods are the same for every Object, however since some Objects may not have joints or more than one link
methods related to these will not work.

```python
milk.set_position(Pose([1, 0, 0]))
```

Since everything inside the BulletWorld is an Object, even a complex environment Object like the kitchen can be spawned
in the same way as the milk.

```python
kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
```

## Costmaps

Costmaps are a way to get positions with respect to certain criterias.
The currently available costmaps are:

* {class}`~pycram.costmaps.OccupancyCostmap`
* {class}`~pycram.costmaps.VisibilityCostmap`
* {class}`~pycram.costmaps.SemanticCostmap`
* {class}`~pycram.costmaps.GaussianCostmap`

It is also possible to merge multiple costmaps to combine different criteria.

### Visibility Costmaps

Visibility costmaps determine every position, around a target position, from which the target is visible. Visibility
Costmaps are able to work with cameras that are movable in height for example, if the robot has a movable torso.

```python
import pycram.costmaps as cm

v = cm.VisibilityCostmap(1.27, 1.60, size=300, resolution=0.02, origin=Pose([0, 0, 0.1], [0, 0, 0, 1]))
```

```python
v.visualize()
```

```python
v.close_visualization()
```

### Occupancy Costmap

Is valid for every position where the robot can be placed without colliding with an object.

```python
o = cm.OccupancyCostmap(0.2, from_ros=False, size=300, resolution=0.02, origin=Pose([0, 0, 0.1], [0, 0, 0, 1]))
```

```python
s = cm.SemanticCostmap(kitchen, "kitchen_island_surface", resolution=0.02)

g = cm.GaussianCostmap(200, 15, resolution=0.02)
```

You can visualize the costmap in the BulletWorld to get an impression what information is actually contained in the
costmap. With this you could also check if the costmap was created correctly.
Visualization can be done via the 'visualize' method of each costmap.

```python
o.visualize()
```

```python
o.close_visualization()
```

It is also possible to combine two costmap, this will result in a new costmap with the same size which contains the
information of both previous costmaps. Combination is done by checking for each position in the two costmaps if they are
zero, in this case to same position in the new costmap will also be zero in any other case the new position will be the
normalized product of the two combined costmaps.

```python
ov = o + v
```

```python
ov.visualize()
```

```python
ov.close_visualization()
```

## Bullet World Reasoning

Allows for geometric reasoning in the BulletWorld. At the moment the following types of reasoning are supported:

* {meth}`~pycram.world_reasoning.stable`
* {meth}`~pycram.world_reasoning.contact`
* {meth}`~pycram.world_reasoning.visible`
* {meth}`~pycram.world_reasoning.occluding`
* {meth}`~pycram.world_reasoning.reachable`
* {meth}`~pycram.world_reasoning.blocking`
* {meth}`~pycram.world_reasoning.supporting`

We start with testing for visibility

```python
from pycram.world_reasoning import visible

milk.set_position(Pose([1, 0, 1]))
visible = visible(milk, pr2.get_link_pose("wide_stereo_optical_frame"))
print(f"Milk visible: {visible}")
```

```python
from pycram.world_reasoning import contact
from pycram.datastructures.world import World

milk.set_position(Pose([1, 0, 0.05]))

plane = World.current_world.objects[0]
contact = contact(milk, plane)
print(f"Milk is in contact with the floor: {contact}")
```

```python
from pycram.world_reasoning import reachable

milk.set_position(Pose([0.6, -0.5, 0.7]))

reachable = reachable(milk, pr2, "r_gripper_tool_frame")
print(f"Milk is reachable for the PR2: {reachable}")
```

# Designators

Designators are symbolic descriptions of Actions, Motions, Objects or Locations. In PyCRAM the different types of
designators are represented by a class which takes a description, the description then tells the designator what to do.

For example, let's look at a Motion Designator to move the robot to a specific location.

## Motion Designators

When using a Motion Designator you need to specify which Process Module needs to be used, either the Process Module for
the real or the simulated robot. A Process Module is the interface between a real or simulated robot, and PyCRAM
designators. By exchanging the Process Module, one can quickly change the robot the plan is executed on, allowing PyCRAM
plans to be re-used across multiple robot platforms. This can be done either with a decorator which can be added to a
function and then every designator executed within this function, will be executed on the specified robot. The other
possibility is a "with" scope which wraps a code piece.

These two ways can also be combined, you could write a function which should be executed on the real robot and the
function contains a "with" scope which executes something on the simulated robot for reasoning purposes.

```python
from pycram.designators.motion_designator import *
from pycram.process_module import simulated_robot, with_simulated_robot

description = MoveMotion(target=PoseSteamped.from_list([1, 0, 0], [0, 0, 0, 1]))

with simulated_robot:
    description.perform()


```

```python
from pycram.process_module import with_simulated_robot


@with_simulated_robot
def move():
    MoveMotion(target=Pose([0, 0, 0], [0, 0, 0, 1])).perform()


move()
```

Other implemented Motion Designator descriptions are:

* Accessing
* Move TCP
* Looking
* Move Gripper
* Detecting
* Move Arm Joint
* World State Detecting

## Object Designators

An Object Designator represents objects. These objects could either be from the BulletWorld or the real world. Object
Designators are used, for example, by the PickUpAction to know which object should be picked up.

```python
from pycram.designators.object_designator import *

milk_desig = BelieveObject(names=["milk"])
milk_desig.resolve()
```

## Location Designator

Location Designator can create a position in cartisian space from a symbolic desctiption

```python
from pycram.designators.object_designator import *

milk_desig = BelieveObject(names=["milk"])
milk_desig.resolve()
```

## Location Designators

Location Designators can create a position in cartesian space from a symbolic description.

```python
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *

robot_desig = BelieveObject(types=[Robot]).resolve()
milk_desig = BelieveObject(names=["milk"]).resolve()
location_desig = CostmapLocation(target=milk_desig, visible_for=robot_desig)

print(f"Resolved: {location_desig.resolve()}")
print()

for pose in location_desig:
    print(pose)

```

# Action Designator

Action Designators are used to describe high-level actions. Action Designators are usually composed of other Designators
to describe the high-level action in detail.

```python
from pycram.designators.action_designator import *
from pycram.datastructures.enums import Arms

with simulated_robot:
    ParkArmsActionDescription([Arms.BOTH]).resolve().perform()

```

# Making a simple plan

To get familiar with the PyCRAM Framework we will write a simple pick and place plan. This plan will let the robot grasp
a cereal box from the kitchen counter and place it on the kitchen island. This is a simple pick and place plan.

```python
from pycram.datastructures.enums import Grasp, TorsoState, Arms
from pycram.datastructures.pose import GraspDescription

cereal_desig = ObjectDesignatorDescription(names=["cereal"])
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
robot_desig = ObjectDesignatorDescription(names=["pr2"])
with simulated_robot:
    ParkArmsActionDescription(Arms.BOTH).resolve().perform()

    MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()

    pickup_arm = Arms.RIGHT
    pickup_pose = CostmapLocation(target=cereal_desig, reachable_for=robot_desig, reachable_arm=pickup_arm)
    
    NavigateActionDescription(target_location=pickup_pose).resolve().perform()
    
    grasp_description = GraspDescription(Grasp.FRONT, None, False)
    PickUpActionDescription(object_designator=cereal_desig, arm=[pickup_arm], grasp_description=grasp_description).resolve().perform()

    ParkArmsActionDescription([Arms.BOTH]).resolve().perform()

    place_island = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve(),
                                           cereal_desig.resolve())

    place_stand = CostmapLocation(place_island, reachable_for=robot_desig, reachable_arm=[pickup_arm], object_in_hand=cereal_desig.resolve())

    NavigateActionDescription(target_location=place_stand).resolve().perform()

    PlaceActionDescription(cereal_desig, target_location=place_island, arm=[pickup_arm]).resolve().perform()

    ParkArmsActionDescription([Arms.BOTH]).resolve().perform()



```

# Task Trees

Task trees are a hierarchical representation of all Actions involved in a plan. The Task tree can later be used to
inspect and restructure the execution order of Actions in the plan.

```python
import pycram.tasktree
import anytree

tt = pycram.tasktree.task_tree
print(anytree.RenderTree(tt))
```

```python
from anytree.dotexport import RenderTreeGraph, DotExporter

RenderTreeGraph(tt).to_picture("tree.png")
```

# ORM

```python
import sqlalchemy.orm
import pycram.orm.base
import pycram.orm.action_designator

# set description of what we are doing
pycram.orm.base.ProcessMetaData().description = "Tutorial for getting familiar with the ORM."

engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
session = sqlalchemy.orm.Session(bind=engine)
pycram.orm.base.Base.metadata.create_all(engine)
session.commit()

tt.insert(session)
```

```python
from sqlalchemy import select

navigations = session.scalars(select(pycram.orm.action_designator.NavigateAction)).all()
print(*navigations, sep="\n")
```

```python
navigations = (session.scalars(
    select(pycram.orm.action_designator.NavigateAction, pycram.orm.base.Position, pycram.orm.base.Quaternion).
    join(pycram.orm.action_designator.NavigateAction.pose).
    join(pycram.orm.base.PoseStamped.position).
    join(pycram.orm.base.PoseStamped.orientation)).all())
print(*navigations, sep="\n")
```

The world can also be closed with the 'exit' method

```python
world.exit()
```
