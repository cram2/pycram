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

# TaskTree Tutorial

In this tutorial we will walk through the capabilities of task trees in pycram.

First we have to import the necessary functionality from pycram.

```python
from pycram.bullet_world import BulletWorld
from pycram.robot_descriptions import robot_description
import pycram.task
from pycram.resolver.plans import Arms
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.designators.object_designator import *
import anytree
import pycram.failures
```

Next we will create a bullet world with a PR2 in a kitchen containing milk and cereal.

```python
world = BulletWorld()
robot = Object(robot_description.name, "robot", robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=['pr2']).resolve()
apartment = Object("apartment", "environment", "apartment.urdf", position=[-1.5, -2.5, 0])
apartment_desig = ObjectDesignatorDescription(names=['apartment']).resolve()
table_top = apartment.get_link_position("cooktop")
# milk = Object("milk", "milk", "milk.stl", position=[table_top[0]-0.15, table_top[1], table_top[2]])
# milk.set_position(position=milk.get_position(), base=True)
# cereal = Object("cereal", "cereal", "breakfast_cereal.stl", position=table_top)
# cereal.set_position(position=[table_top[0]-0.1, table_top[1] + 0.5, table_top[2]], base=True)
# milk_desig = ObjectDesignator(ObjectDesignatorDescription(name="milk", type="milk"))
# cereal_desig = ObjectDesignator(ObjectDesignatorDescription(name="cereal", type="cereal"))
```

```python
import numpy as np


def get_n_random_positions(pose_list, n=4, dist=0.5, random=True):
    positions = [pos[0] for pos in pose_list[:1000]]
    all_indices = list(range(len(positions)))
    print(len(all_indices))
    pos_idx = np.random.choice(all_indices) if random else all_indices[0]
    all_indices.remove(pos_idx)
    n_positions = np.zeros((n, 3))
    for i in range(n):
        n_positions[i, :] = positions[pos_idx]
    found_count = 1
    found_indices = [pos_idx]
    for i in range(len(positions) - 1):
        pos_idx = np.random.choice(all_indices) if random else all_indices[i]
        diff = np.absolute(np.linalg.norm(n_positions - positions[pos_idx], axis=1))
        # print(diff)
        min_diff = np.min(diff)
        # print(min_diff)
        if min_diff >= dist:
            # print("found")
            n_positions[found_count, :] = positions[pos_idx]
            found_indices.append(pos_idx)
            found_count += 1
        all_indices.remove(pos_idx)
        if found_count == n:
            break
    found_poses = [pose_list[i] for i in found_indices]
    # found_positions = [positions[i] for i in found_indices]
    # for i in range(len(found_positions)):
    #     print(found_poses[i][0])
    #     print(found_positions[i])
    #     assert np.allclose(found_positions[i],found_poses[i][0])
    # for i in range(len(found_poses)):
    #     for j in range(i+1,len(found_poses)):
    #         pos1 = np.array(found_poses[i][0])
    #         pos2 = np.array(found_poses[j][0])
    #         diff = np.absolute(np.linalg.norm(pos1 - pos2))
    #         print(diff)
    #         assert diff >= dist
    return found_poses



```

```python
from pycram.costmaps import SemanticCostmap
from pycram.pose_generator_and_validator import pose_generator

scm = SemanticCostmap(apartment, "island_countertop")
poses_list = list(pose_generator(scm, number_of_samples=-1))
poses_list.sort(reverse=True, key=lambda x: np.linalg.norm(x[0]))
object_poses = get_n_random_positions(poses_list)
object_names = ["bowl", "milk", "breakfast_cereal", "spoon"]
objects = {}
object_desig = {}
for obj_name, obj_pose in zip(object_names, object_poses):
    print(obj_name)
    print(obj_pose)
    objects[obj_name] = Object(obj_name, obj_name, obj_name + ".stl",
                               position=[obj_pose[0][0], obj_pose[0][1], table_top[2]])
    objects[obj_name].move_base_to_origin_pos()
    objects[obj_name].original_pose = objects[obj_name].get_position_and_orientation()
    object_desig[obj_name] = ObjectDesignatorDescription(names=[obj_name], types=[obj_name]).resolve()
print(object_poses)
```

If You want to visualize all apartment frames

```python
import pybullet as p

for link_name in apartment.links.keys():
    world.add_vis_axis(apartment.get_link_pose(link_name))
    p.addUserDebugText(link_name, apartment.get_link_position(link_name))
```

```python
world.remove_vis_axis()
p.removeAllUserDebugItems()
```

Finally, we create a plan where the robot parks his arms, walks to the kitchen counter and picks the thingy. Then we
execute the plan.

```python
from pycram.external_interfaces.ik import IKError


@pycram.task.with_tree
def plan(obj, obj_desig, torso=0.2, place="countertop"):
    world.reset_bullet_world()
    with simulated_robot:
        ParkArmsActionPerformable(Arms.BOTH).perform()

        MoveTorsoActionPerformable(torso).perform()
        location = CostmapLocation(target=obj_desig, reachable_for=robot_desig)
        pose = location.resolve()
        print()
        NavigateActionPerformable(pose.pose).perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()
        good_torsos.append(torso)
        picked_up_arm = pose.reachable_arms[0]
        PickUpActionPerformable(object_designator=obj_desig, arm=pose.reachable_arms[0], grasp="front").perform()

        ParkArmsActionPerformable(Arms.BOTH).perform()
        scm = SemanticCostmapLocation(place, apartment_desig, obj_desig)
        pose_island = scm.resolve()

        place_location = CostmapLocation(target=pose_island.pose, reachable_for=robot_desig,
                                         reachable_arm=picked_up_arm)
        pose = place_location.resolve()

        NavigateActionPerformable(pose.pose).perform()

        PlaceActionPerformable(object_designator=obj_desig, target_location=pose_island.pose,
                               arm=picked_up_arm).perform()

        ParkArmsActionPerformable(Arms.BOTH).perform()


good_torsos = []
for obj_name in object_names:
    done = False
    torso = 0.25 if len(good_torsos) == 0 else good_torsos[-1]
    while not done:
        try:
            plan(objects[obj_name], object_desig[obj_name], torso=torso, place="island_countertop")
            done = True
            objects[obj_name].original_pose = objects[obj_name].get_position_and_orientation()
        except (StopIteration, IKError) as e:
            print(type(e))
            print(e)
            print("no solution")
            torso += 0.05
            if torso > 0.3:
                break
print(good_torsos)
```

Now we get the task tree from its module and render it. Rendering can be done with any render method described in the
anytree package. We will use ascii rendering here for ease of displaying.

```python
tt = pycram.task.task_tree
print(anytree.RenderTree(tt, style=anytree.render.AsciiStyle()))
```

As we see every task in the plan got recorded correctly. It is noticeable that the tree begins with a NoOperation node.
This is done because several, not connected, plans that get executed after each other should still appear in the task
tree. Hence, a NoOperation node is the root of any tree. If we re-execute the plan we would see them appear in the same
tree even though they are not connected.

```python
world.reset_bullet_world()
plan()
print(anytree.RenderTree(tt, style=anytree.render.AsciiStyle()))
```

Projecting a plan in a new environment with its own task tree that only exists while the projected plan is running can
be done with the ``with`` keyword. When this is done, both the bullet world and task tree are saved and new, freshly
reset objects are available. At the end of a with block the old state is restored. The root for such things is then
called ``simulation()``.

```python
with pycram.task.SimulatedTaskTree() as stt:
    print(anytree.RenderTree(pycram.task.task_tree, style=anytree.render.AsciiStyle()))
print(anytree.RenderTree(pycram.task.task_tree, style=anytree.render.AsciiStyle()))
```

Task tree can be manipulated with ordinary anytree manipulation. If we for example want to discard the second plan, we
would write:

```python
tt.root.children = (tt.root.children[0],)
print(anytree.RenderTree(tt, style=anytree.render.AsciiStyle()))
```

We can now re-execute this (modified) plan by executing the leaf in pre-ordering iteration using the anytree
functionality. This will not append the re-execution to the task tree.

```python
world.reset_bullet_world()
with simulated_robot:
    [node.code.execute() for node in tt.root.leaves]
print(anytree.RenderTree(pycram.task.task_tree, style=anytree.render.AsciiStyle()))
```

Nodes in the task tree contain additional information about the status and time of a task.

```python
print(pycram.task.task_tree.children[0])
```

The task tree can also be reset to an empty one by invoking:

```python
pycram.task.reset_tree()
print(anytree.RenderTree(pycram.task.task_tree, style=anytree.render.AsciiStyle()))
```

If a plan fails using the PlanFailure exception, the plan will not stop. Instead, the error will be logged and saved in
the task tree as a failed subtask. First let's create a simple failing plan and execute it.

```python
@pycram.task.with_tree
def failing_plan():
    raise pycram.plan_failures.PlanFailure("Oopsie!")


failing_plan()
```

We can now investigate the nodes of the tree, and we will see that the tree indeed contains a failed task.

```python
print(anytree.RenderTree(pycram.task.task_tree, style=anytree.render.AsciiStyle()))
print(pycram.task.task_tree.children[0])
```

```python
world.exit()
```
