---
jupyter:
  jupytext:
    formats: ipynb,md
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.17.0
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

# Motion Planning with Graphs of Convex Sets
This notebook shows how PyCRAMs Graph of Convex Sets (GCS) functionality can be used to plan an end effector path in a complex environment and how to execute that path.
Paths will be represented as sequences of waypoints. To execute a sequence of waypoints as one motion the MoveTCPWaypointsMotion Designator is introduced which leverages [Giskard](https://github.com/SemRoCo/giskardpy) to calculate the motion of the robot. To follow the below examples Giskard should be started with this command `roslaunch giskardpy giskardpy_pr2_standalone.launch`.
The following three cells initilaize the PyCRAM world with a kitchen environment and the PR2 robot, and sync it with the world of Giskard. Then a large drawer is opened, the robot is teleported close to that drawer and it's arms are parked.

```python
from geometry_msgs.msg import PoseStamped
from pycram.graph_of_convex_sets import BoundingBox, GraphOfConvexSets
from pycram.datastructures.enums import WorldMode
from pycrap.ontologies import Robot, Kitchen
from pycram.worlds.bullet_world import BulletWorld

from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.ros_utils.robot_state_updater import WorldStateUpdater
from tf.transformations import quaternion_from_matrix
from pycram.robot_plans import *
from pycram.process_module import real_robot
from pycram.external_interfaces.giskard import sync_worlds
from pycram.robot_plans import *
from pycram.robot_plans.motions import MoveTCPWaypointsMotion
from geometry_msgs.msg import Quaternion

world = BulletWorld(mode=WorldMode.GUI)

viz_marker_publisher = VizMarkerPublisher()

kitchen = Object("kitchen", Kitchen, "kitchen.urdf")

robot = Object("pr2", Robot, f"pr2.urdf", pose=Pose([0, 1, 0]))
WorldStateUpdater(tf_topic="/tf", joint_state_topic="/joint_states")
sync_worlds()
```

```python
from pycram.external_interfaces.giskard import giskard_wrapper

kitchen.set_joint_position("oven_area_area_right_drawer_joint", .45)
giskard_wrapper.set_seed_configuration({"oven_area_area_right_drawer_joint": .45})
pose = PoseStamped()
pose.header.reference_frame = 'map'
pose.pose.position.y = 1
pose.pose.orientation.w = 1
giskard_wrapper.monitors.add_set_seed_odometry(pose)
giskard_wrapper.execute()
```

```python
from pycram.datastructures.enums import Arms
with real_robot:
    try:
        ParkArmsActionDescription([Arms.BOTH]).resolve().perform()
    except:
        pass
```

Now, we define a search space for the GCS algorithm around the open drawer and the robot, and calculate the connectivity graph.

```python
search_space = BoundingBox(min_x=-0.0, max_x=1.5, min_y=0.1, max_y=2, min_z=0, max_z=1.45)
print('Calculating graph of convex sets...')
cg = GraphOfConvexSets.free_space_from_world(world, search_space=search_space)
```

Given the connectivity graph a start pose and a goal pose for the path search is needed.
The start pose is defined as the current pose of the right gripper of the robot.
For the goal pose two possible positions at different levels in the drawer are predefined. These can be (un)commented to search for paths from one level of the drawer to another.
For the orientation of the goal pose, an orientation is chosen that makes sense when reaching into the drawer from the right side.
As the found path consist of poses with identity orientation we copy our goal orientation directly into the last pose in the path.
Here we only care about the orientation for the last pose in the path, because we do not want to constrain the motion generation with end effector orientations along the path, but still want to have the desired orientation at the end of the path.

```python
print('reading current pose of the gripper...')
start_pose = robot.links['r_gripper_tool_frame'].pose
print('searching for path to goal pose...')
start = Pose([start_pose.position.x, start_pose.position.y, start_pose.position.z])
# goal_position = Pose([1, 1.49, 1.0])
goal_position = Pose([1, 1.49, 0.7])
goal_orientation = quaternion_from_matrix([[0, -1, 0, 0],
                                           [1, 0, 0, 0],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])
path = cg.path_from_to(start, goal_position)
path[-1].pose.orientation = Quaternion(*goal_orientation)
print('done')
```

The MoveTCPWaypointsMotion Designator moves the specified arm to each position in the given path in the order of the list of poses. Therefore it takes as necessary inputs a path of poses and which arm should be used. The other parameters default to the shown values. The ENFORCE_ORIENTATION_FINAL_POINT movement type ensures that only the orientation of the last pose in the list is achieved. It can be changed to ENFORCE_ORIENTATION_STRICT to achieve the orientation for each pose in the list.
For the path that we provide here the first value is skipped as that is equal to the start pose from earlier.

```python
print('move along path to goal pose...')
from pycram.datastructures.enums import WaypointsMovementType
with real_robot:
    MoveTCPWaypointsMotion(path[1:], Arms.RIGHT, movement_type=WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT, allow_gripper_collision=False).perform()
```

Alternatively, before executing the planned path with the MoveTCPWaypointsMotion Designator the path could be further improved by postprocessing the output from the GCS path finding algorithm.
For example below is a simpler filter algorithm that removes each waypoint from the path that has a distance from its predecessor below a threshold parameter.

```python
import numpy as np
print('path refinement...')
print('Original Path:', path)
print(len(path))
def filter_path(path, distance=0.05):
    new_path = []
    p_start = start
    new_path.append(start)
    for p in path[:-1]:
        dist = np.linalg.norm([p.position_as_list()[0] - p_start.position_as_list()[0], p.position_as_list()[1] - p_start.position_as_list()[1], p.position_as_list()[2] - p_start.position_as_list()[2]])
        if dist > distance:
            new_path.append(p)
        p_start = p
    new_path.append(path[-1])
    return new_path

new_path = filter_path(path)
print('New Path:', new_path)
print(len(new_path))

```

```python
print('move along path to goal pose...')
with real_robot:
    MoveTCPWaypointsMotion(filter_path(path), Arms.RIGHT).perform()
```
