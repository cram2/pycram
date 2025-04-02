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

# TaskTree Tutorial

In this tutorial we will walk through the capabilities of task trees in pycram.

First we have to import the necessary functionality from pycram.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.robot_description import RobotDescription
import pycram.tasktree
from pycram.datastructures.enums import Arms, ObjectType
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.designators.object_designator import *
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
import anytree
import pycram.failures
from pycram.datastructures.pose import GraspDescription
```

Next we will create a bullet world with a PR2 in a kitchen containing milk and cereal.

```python
from pycrap.ontologies import Milk, Cereal, Robot, Kitchen
world = BulletWorld(WorldMode.DIRECT)
pr2 = Object("pr2", Robot, "pr2.urdf")
kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
milk = Object("milk", Milk, "milk.stl", pose=Pose([1.3, 1, 0.9]))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl", pose=Pose([1.3, 0.7, 0.95]))
milk_desig = ObjectDesignatorDescription(names=["milk"])
cereal_desig = ObjectDesignatorDescription(names=["cereal"])
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
```

Finally, we create a plan where the robot parks his arms, walks to the kitchen counter and picks the cereal and places it on the table. Then we execute the plan.

```python
@pycram.tasktree.with_tree
def plan():
    with simulated_robot:
        ParkArmsAction(Arms.BOTH).perform()
        MoveTorsoActionDescription([TorsoState.MID]).resolve().perform()
        pickup_pose = CostmapLocation(target=cereal_desig.resolve(), reachable_for=robot_desig).resolve()
        pickup_arm = pickup_pose.arm
        NavigateActionDescription(target_location=pickup_pose).resolve().perform()
        grasp = pickup_pose.grasp_description
        PickUpActionDescription(object_designator=cereal_desig, arm=[pickup_arm], grasp_description=grasp).resolve().perform()
        ParkArmsActionDescription([Arms.BOTH]).resolve().perform()

        place_island = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve(),
                                           cereal_desig.resolve()).resolve()

        place_stand = CostmapLocation(place_island, reachable_for=robot_desig, reachable_arm=[pickup_arm],  object_in_hand=cereal_desig.resolve())

        NavigateActionDescription(target_location=place_stand).resolve().perform()

        PlaceActionDescription(cereal_desig, target_location=place_island, arm=[pickup_arm]).resolve().perform()

        ParkArmsActionDescription([Arms.BOTH]).resolve().perform()

        ParkArmsAction(Arms.BOTH).perform()

plan()

```

Now we get the task tree from its module and render it. Rendering can be done with any render method described in the anytree package. We will use ascii rendering here for ease of displaying.

```python
tt = pycram.tasktree.task_tree
print(anytree.RenderTree(tt.root))
```

As we see every task in the plan got recorded correctly. It is noticeable that the tree begins with a NoOperation node. This is done because several, not connected, plans that get executed after each other should still appear in the task tree. Hence, a NoOperation node is the root of any tree. If we re-execute the plan we would see them appear in the same tree even though they are not connected.

```python
world.reset_world()
plan()
print(anytree.RenderTree(tt.root))
```

Projecting a plan in a new environment with its own task tree that only exists while the projected plan is running can be done with the ``with`` keyword. When this is done, both the bullet world and task tree are saved and new, freshly reset objects are available. At the end of a with block the old state is restored. The root for such things is then called ``simulation()``.

```python
with pycram.tasktree.SimulatedTaskTree() as stt:
    print(anytree.RenderTree(pycram.tasktree.task_tree.root))
print(anytree.RenderTree(pycram.tasktree.task_tree.root))
```

Task tree can be manipulated with ordinary anytree manipulation. If we for example want to discard the second plan, we would write

```python
tt.root.children = (tt.root.children[0],)
print(anytree.RenderTree(tt.root, style=anytree.render.AsciiStyle()))
```
We can now re-execute this (modified) plan by executing the leaf in pre-ordering iteration using the anytree functionality. This will not append the re-execution to the task tree.

```python
world.reset_world()
with simulated_robot:
    [node.action.perform() for node in tt.root.leaves]
print(anytree.RenderTree(pycram.tasktree.task_tree, style=anytree.render.AsciiStyle()))
```

Nodes in the task tree contain additional information about the status and time of a task.

```python
print(pycram.tasktree.task_tree.children[0])
```

The task tree can also be reset to an empty one by invoking

```python
pycram.tasktree.task_tree.reset_tree()
print(anytree.RenderTree(pycram.tasktree.task_tree, style=anytree.render.AsciiStyle()))
```

If a plan fails using the PlanFailure exception, the plan will not stop. Instead, the error will be logged and saved in the task tree as a failed subtask. First let's create a simple failing plan and execute it.

```python
@pycram.tasktree.with_tree
def failing_plan():
    raise pycram.failures.PlanFailure("Oopsie!")

try:
    failing_plan()
except pycram.failures.PlanFailure as e:
    print(e)
```

We can now investigate the nodes of the tree, and we will see that the tree indeed contains a failed task.

```python
print(anytree.RenderTree(pycram.tasktree.task_tree, style=anytree.render.AsciiStyle()))
print(pycram.tasktree.task_tree.children[0])
```

```python
world.exit()
```
