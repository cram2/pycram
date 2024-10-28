---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.3
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

# ORM querying examples

In this tutorial, we will get to see more examples of ORM querying. 


First, we will gather a lot of data. In order to achieve that we will write a randomized experiment for grasping a couple of objects.
In the experiment the robot will try to grasp a randomized object using random poses and torso heights.

```python
from tf import transformations
import itertools
from typing import Optional, List, Tuple
import numpy as np
import sqlalchemy.orm
import tqdm
import pycram.orm.base
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object as BulletWorldObject
from pycram.designators.action_designator import MoveTorsoAction, PickUpAction, NavigateAction, ParkArmsAction, ParkArmsActionPerformable, MoveTorsoActionPerformable
from pycram.designators.object_designator import ObjectDesignatorDescription
from pycram.failures import PlanFailure
from pycram.process_module import ProcessModule
from pycram.datastructures.enums import Arms, ObjectType, Grasp, WorldMode
from pycram.process_module import simulated_robot
from pycram.orm.action_designator import PickUpAction as ORMPickUpAction
from pycram.orm.base import RobotState, Position, ProcessMetaData, Pose as ORMPose
from pycram.orm.tasktree import TaskTreeNode
from pycram.orm.object_designator import Object
from pycram.tasktree import task_tree, TaskTree
import pycram.orm
import sqlalchemy.sql
import pandas as pd

from pycram.datastructures.pose import Pose

np.random.seed(420)

ProcessModule.execution_delay = False
ProcessMetaData().description = "Tutorial for learning from experience in a Grasping action."


class GraspingExplorer:
    """Class to try randomized grasping plans."""

    world: Optional[BulletWorld]

    def __init__(self, robots: Optional[List[Tuple[str, str]]] = None, objects: Optional[List[Tuple[str, str]]] = None,
                 arms: Optional[List[Arms]] = None, grasps: Optional[List[Grasp]] = None,
                 samples_per_scenario: int = 1000):
        """
        Create a GraspingExplorer.
        :param robots: The robots to use
        :param objects: The objects to try to grasp
        :param arms: The arms of the robot to use
        :param grasps: The grasp orientations to use
        :param samples_per_scenario: The number of tries per scenario.
        """
        # store exploration space
        if not robots:
            self.robots: List[Tuple[str, str]] = [("pr2", "pr2.urdf")]

        if not objects:
            self.objects: List[Tuple[str, ObjectType, str]] = [
                ("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl"),
                ("bowl", ObjectType.BOWL, "bowl.stl"),
                ("milk", ObjectType.MILK, "milk.stl"),
                ("spoon", ObjectType.SPOON, "spoon.stl")]

        if not arms:
            self.arms: List[Arms] = [Arms.LEFT, Arms.RIGHT]

        if not grasps:
            self.grasps: List[Grasp] = [Grasp.LEFT, Grasp.RIGHT, Grasp.FRONT, Grasp.TOP]

        # store trials per scenario
        self.samples_per_scenario: int = samples_per_scenario

        # chain hyperparameters
        self.hyper_parameters = [self.robots, self.objects, self.arms, self.grasps]

        self.total_tries = 0
        self.total_failures = 0

    def perform(self, session: sqlalchemy.orm.Session):
        """
        Perform all experiments.
        :param session: The database-session to insert the samples in.
        """

        # create progress bar
        progress_bar = tqdm.tqdm(
            total=np.prod([len(p) for p in self.hyper_parameters]) * self.samples_per_scenario)

        self.world = BulletWorld(WorldMode.DIRECT)

        # for every robot
        for robot, robot_urdf in self.robots:

            # spawn it
            robot = BulletWorldObject(robot, ObjectType.ROBOT, robot_urdf)

            # for every obj
            for obj, obj_type, obj_stl in self.objects:

                # spawn it
                bw_object = BulletWorldObject(obj, obj_type, obj_stl, pose=Pose([0, 0, 0.75], [0, 0, 0, 1]))

                # create object designator
                object_designator = ObjectDesignatorDescription(names=[obj])

                # for every arm and grasp pose
                for arm, grasp in itertools.product(self.arms, self.grasps):
                    # sample positions in 2D
                    positions = np.random.uniform([-2, -2], [2, 2], (self.samples_per_scenario, 2))

                    # for every position
                    for position in positions:

                        # set z axis to 0
                        position = [*position, 0]

                        # calculate orientation for robot to face the object
                        angle = np.arctan2(position[1], position[0]) + np.pi
                        orientation = list(transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))

                        # try to execute a grasping plan
                        with simulated_robot:
                            ParkArmsActionPerformable(Arms.BOTH).perform()
                            # navigate to sampled position
                            NavigateAction([Pose(position, orientation)]).resolve().perform()

                            # move torso
                            height = np.random.uniform(0., 0.33, 1)[0]
                            MoveTorsoActionPerformable(height).perform()

                            # try to pick it up
                            try:
                                PickUpAction(object_designator, [arm], [grasp]).resolve().perform()

                            # if it fails
                            except PlanFailure:

                                # update failure stats
                                self.total_failures += 1

                            # reset BulletWorld
                            self.world.reset_world()

                            # update progress bar
                            self.total_tries += 1

                            # insert into database
                            task_tree.root.insert(session, use_progress_bar=False)
                            task_tree.reset_tree()

                            progress_bar.update()
                            progress_bar.set_postfix(success_rate=(self.total_tries - self.total_failures) /
                                                                  self.total_tries)

                bw_object.remove()
            robot.remove()

```

Next we have to establish a connection to a database and execute the experiment a couple of times. Note that the (few) number of samples we generate is only for demonstrations.
For robust and reliable machine learning millions of samples are required.


```python
engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:")
session = sqlalchemy.orm.Session(bind=engine)
pycram.orm.base.Base.metadata.create_all(bind=engine)
session.commit()

explorer = GraspingExplorer(samples_per_scenario=10)
explorer.perform(session)
```

The success_rate of the output above indicates how many of our samples succeeded in trying to grasp a randomized object.


Now that we have data to query from and a running session, we can actually start creating queries. 
Let's say we want to select positions of robots that were able to grasp a specific object (in this case a "milk" object):

```python
from sqlalchemy import select
from pycram.datastructures.enums import ObjectType

milk = BulletWorldObject("milk", ObjectType.MILK, "milk.stl")

# query all relative robot positions in regard to an objects position
# make sure to order the joins() correctly
query = (select(ORMPickUpAction.arm, ORMPickUpAction.grasp, RobotState.torso_height, Position.x, Position.y)
         .join(TaskTreeNode.action.of_type(ORMPickUpAction))
         .join(ORMPickUpAction.robot_state)
         .join(RobotState.pose)
         .join(ORMPose.position)
         .join(ORMPickUpAction.object).where(Object.obj_type == milk.obj_type)
                                      .where(TaskTreeNode.status == "SUCCEEDED"))
print(query)

df = pd.read_sql_query(query, session.get_bind())
print(df)
```

If you are not familiar with sqlalchemy querying you might wonder what the of_type() function does and why we needed it in this query:

In order to understand the importance of the of_type() function in the joins above it is crucial to understand the inheritance structure in the ORM package. The action necessary for this query is the PickUpAction. It inherits the Action class/table (which holds all the actions). The Action class itself on the other hand inherits Designator (which holds all the actions, but also all the motions). 
We started our joins by joining TaskTreeNode on its relationship to Code and Code on its relationship to Designator. Next table we need is the PickUpAction table, but there is no specified relationship between Designator and PickUpAction. But we do know that a PickUpAction is actually a Designator, meaning, it inherits from Designator. So we can just "tell" the join to join Code on every Designator, that is "of_type" PickUpAction (.join(Code.designator.of_type(ORMPickUpAction))). 
The effect of this function can also be seen in the printed query of above's output. 


Another interesting query: Let's say we want to select the torso height and positions of robots relative to the object they were trying to grasp:

```python
robot_pose = sqlalchemy.orm.aliased(ORMPose)
object_pose = sqlalchemy.orm.aliased(ORMPose)
robot_position = sqlalchemy.orm.aliased(Position)
object_position = sqlalchemy.orm.aliased(Position)

query = (select(TaskTreeNode.status, Object.obj_type, 
                       sqlalchemy.label("relative torso height", object_position.z - RobotState.torso_height),
                       sqlalchemy.label("x", robot_position.x - object_position.x),
                       sqlalchemy.label("y", robot_position.y - object_position.y))
         .join(TaskTreeNode.action.of_type(ORMPickUpAction))
         .join(ORMPickUpAction.robot_state)
         .join(robot_pose, RobotState.pose)
         .join(robot_position, robot_pose.position)
         .join(ORMPickUpAction.object)
         .join(object_pose, Object.pose)
         .join(object_position, object_pose.position))
print(query)

df = pd.read_sql(query, session.get_bind())
df["status"] = df["status"].apply(lambda x: str(x))
print(df)
```

Obviously the query returned every row of the database since we didn't apply any filters.

Why is this query interesting? This query not only required more joins and the usage of the of_type() function, but we actually needed to access two of the tables twice with different purposes, namely the Pose and Position tables. We wanted to get the position of the robot relative to the object position, meaning we had to obtain all robot positions and all object positions. If we want to access the same table twice, we have to make sure to rename (one of) the occurrences in our query in order to provide proper sql syntax. This can be done by creating aliases using the sqlalchemy.orm.aliased() function. Sqlalchemy will automatically rename all the aliased tables for you during runtime.
