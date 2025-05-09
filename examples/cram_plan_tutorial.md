from pycram.datastructures.pose import PoseStamped---
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

Next we will create a bullet world with a PR2 in a kitchen containing milk and cereal.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.robot_description import RobotDescription
from pycram.datastructures.enums import Arms, ObjectType
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.designators.object_designator import *
from pycram.datastructures.pose import PoseStamped
from pycram.world_concepts.world_object import Object
import pycram.failures
import numpy as np
from pycrap.ontologies import Milk, Cereal, Robot, Kitchen, Spoon, Apartment, Bowl

np.random.seed(4)

use_multiverse = False
viz_marker_publisher = None
if use_multiverse:
    try:
        from pycram.worlds.multiverse import Multiverse

        world = Multiverse()
    except ImportError:
        raise ImportError("Multiverse is not installed, please install it to use it.")
else:
    from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher

    world = BulletWorld()
    viz_marker_publisher = VizMarkerPublisher()

robot = Object("pr2", Robot, "pr2.urdf")
robot_desig = ObjectDesignatorDescription(names=['pr2']).resolve()
apartment = Object("apartment", Apartment, "apartment.urdf")
apartment_desig = ObjectDesignatorDescription(names=['apartment']).resolve()
table_top_name = "stove" if use_multiverse else "cooktop"
table_top = apartment.get_link_position(table_top_name)
```

```python
import numpy as np


def get_n_random_positions(pose_list, n=4, dist=0.5, random=True):
    positions = [pose.position.to_list() for pose in pose_list[:1000]]
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
        min_diff = np.min(diff)
        if min_diff >= dist:
            n_positions[found_count, :] = positions[pos_idx]
            found_indices.append(pos_idx)
            found_count += 1
        all_indices.remove(pos_idx)
        if found_count == n:
            break
    found_poses = [pose_list[i] for i in found_indices]
    return found_poses
```

```python
import pycrap
from pycram.tf_transformations import quaternion_from_euler

from pycram.costmaps import SemanticCostmap
from pycram.pose_generator_and_validator import PoseGenerator

counter_name = "counter_sink_stove" if use_multiverse else "island_countertop"
counter_link = apartment.get_link(counter_name)
counter_bounds = counter_link.get_axis_aligned_bounding_box()
scm = SemanticCostmap(apartment, counter_name)
# take only 6 cms from edges as viable region
edges_cm = scm.get_edges_map(0.06, horizontal_only=True)
poses_list = list(PoseGenerator(edges_cm, number_of_samples=-1))
poses_list.sort(reverse=True, key=lambda x: np.linalg.norm(x.position.to_list()))
object_poses = get_n_random_positions(poses_list)
object_names = ["breakfast_cereal", "milk"]
object_types = [Cereal, Milk]
objects = {}
object_desig = {}
for obj_name, obj_type, obj_pose in zip(object_names, object_types, object_poses):
    if obj_pose.position.x > counter_link.position.x:
        z_angle = np.pi
    else:
        z_angle = 0
    orientation = quaternion_from_euler(0, 0, z_angle)
    objects[obj_name] = Object(obj_name, obj_type, obj_name + ".stl",
                               pose=PoseStamped.from_list([obj_pose.position.x, obj_pose.position.y, table_top.z],
                                         orientation))
    objects[obj_name].move_base_to_origin_pose()
    objects[obj_name].original_pose = objects[obj_name].pose
    object_desig[obj_name] = ObjectDesignatorDescription(names=[obj_name], types=[obj_type]).resolve()
world.update_original_state()
```

If You want to visualize all apartment frames

```python
import pycram_bullet as p

for link_name in apartment.links.keys():
    world.add_vis_axis(apartment.get_link_pose(link_name))
    #p.addUserDebugText(link_name, apartment.get_link_position(link_name), physicsClientId=world.id)
```

```python
world.remove_vis_axis()
#p.removeAllUserDebugItems()
```

Finally, we create a plan where the robot parks his arms, walks to the kitchen counter and picks the thingy. Then we
execute the plan.

```python
from pycram.external_interfaces.ik import IKError
from pycram.datastructures.enums import Grasp
from pycram.datastructures.grasp import GraspDescription


def plan(obj_desig: Object, torso=None, place=counter_name):
    world.reset_world()
    with simulated_robot:
        ParkArmsAction(Arms.BOTH).perform()
        if torso is None:
            torso={"torso_lift_joint": 0.2}
        MoveJointsMotion(list(torso.keys()), list(torso.values())).perform()
        grasp = GraspDescription(Grasp.FRONT, Grasp.TOP if issubclass(obj_desig.obj_type, Spoon) else None,
                                 False)        
        pickup_arm = Arms.RIGHT
        location = CostmapLocation(target=obj_desig, reachable_for=robot_desig, grasp_descriptions=[grasp], reachable_arm=pickup_arm)
        print()
        NavigateActionDescription(location, True).resolve().perform()
        ParkArmsAction(Arms.BOTH).perform()
        good_torsos.append(torso)

        PickUpAction(object_designator=obj_desig, arm=pickup_arm,  grasp_description=grasp).perform()
        ParkArmsAction(Arms.BOTH).perform()
        scm = SemanticCostmapLocation(place, apartment_desig, obj_desig, horizontal_edges_only=True,
                                      edge_size_in_meters=0.08)
        scm_iter = iter(scm)
        n_retries = 0
        found = False
        while not found:
            try:
                print(f"Trying {n_retries} to find a place")
                if n_retries == 0:
                    pose_island = next(scm_iter)
                    cost_map_size = len(np.where(scm.sem_costmap.map > 0)[0])
                    print(f"cost_map_size: {cost_map_size}")
                else:
                    for _ in range(np.random.randint(100, 110)):
                        pose_island = next(scm_iter)
                print("found pose_island")
                if pose_island.pose.position.x > counter_link.position.x:
                    z_angle = np.pi
                else:
                    z_angle = 0
                orientation = quaternion_from_euler(0, 0, z_angle)
                pose_island = PoseStamped.from_list(pose_island.position.to_list(), orientation)
                pose_island.position.z += 0.07
                print(pose_island.position)
                place_location = CostmapLocation(target=pose_island, reachable_for=robot_desig,  reachable_arm=[pickup_arm])
                pose = place_location.resolve()
                NavigateAction(pose, True).perform()
                PlaceAction(object_designator=obj_desig, target_location=pose_island,
                                       arm=pickup_arm).perform()
                found = True
            except (StopIteration, IKError) as e:
                print("Retrying")
                print(e)
                n_retries += 1
                if n_retries > 3:
                    raise StopIteration("No place found")

        ParkArmsAction(Arms.BOTH).perform()

good_torsos = []
for obj_name in object_names:
    done = False
    torso = {"torso_lift_joint": 0.25} if len(good_torsos) == 0 else good_torsos[-1]
    while not done:
        try:
            plan(object_desig[obj_name], torso=torso, place=counter_name)
            done = True
            print(f"Object {obj_name} placed")
            objects[obj_name].original_pose = objects[obj_name].pose
            world.update_original_state()
        except (StopIteration, IKError) as e:
            print(type(e))
            print("no solution")
            torso["torso_lift_joint"] += 0.05
            if torso["torso_lift_joint"] > 0.3:
                break
print(good_torsos)
```

Now we get the task tree from its module and render it. Rendering can be done with any render method described in the
anytree package. We will use ascii rendering here for ease of displaying.

```python
if viz_marker_publisher is not None:
    viz_marker_publisher._stop_publishing()
world.exit()
```
