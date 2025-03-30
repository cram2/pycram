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

# Hands on Object Relational Mapping in PyCram

This tutorial will walk you through the serialization of a minimal plan in pycram.
First we will import sqlalchemy, create an in-memory database and connect a session to it.

```python
import sqlalchemy.orm

engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
session = sqlalchemy.orm.Session(bind=engine)
session
```

Next we create the database schema using the sqlalchemy functionality. For that we need to import the base class of pycram.orm.

```python
import pycram.orm.action_designator
pycram.orm.base.Base.metadata.create_all(engine)
session.commit()
```

Next we will write a simple plan where the robot parks his arms and then moves somewhere. We will construct a TaskTree around it such that we can serialize it later. As usual, we first create a world and then define the plan. By doing so, we obtain the task tree.

```python
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import Arms, ObjectType, Grasp, WorldMode, TorsoState
from pycram.tasktree import with_tree
import pycram.tasktree
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.designators.object_designator import *
from pycram.datastructures.pose import PoseStamped
from pycram.orm.base import ProcessMetaData
import anytree
from pycrap.ontologies import Robot, Kitchen, Milk, Cereal

world = BulletWorld(WorldMode.DIRECT)
pr2 = Object("pr2", Robot, "pr2.urdf")
kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([1.3, 1, 0.9]))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl", pose=PoseStamped.from_list([1.3, 0.7, 0.95]))
milk_desig = ObjectDesignatorDescription(names=["milk"])
cereal_desig = ObjectDesignatorDescription(names=["cereal"])
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])


@with_tree
def plan():
    with simulated_robot:
        ParkArmsAction(Arms.BOTH).perform()
        MoveTorsoActionDescription([TorsoState.MID]).resolve().perform()
        pickup_pose = CostmapLocation(target=cereal_desig.resolve(), reachable_for=robot_desig).resolve()
        pickup_arm = Arms.RIGHT
        NavigateActionDescription(target_location=[pickup_pose]).resolve().perform()
        grasp = pickup_pose.grasp_description
        PickUpActionDescription(object_designator=cereal_desig, arm=pickup_arm,
                                grasp_description=[grasp]).resolve().perform()
        ParkArmsActionDescription([Arms.BOTH]).resolve().perform()

        place_island = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve(),
                                               cereal_desig.resolve()).resolve()

        place_stand = CostmapLocation(place_island, reachable_for=robot_desig, reachable_arm=[pickup_arm],
                                      object_in_hand=cereal_desig.resolve()).resolve()

        NavigateActionDescription(target_location=place_stand).resolve().perform()

        PlaceActionDescription(cereal_desig, target_location=place_island, arm=pickup_arm).resolve().perform()

        ParkArmsAction(Arms.BOTH).perform()


plan()

# set description of what we are doing
ProcessMetaData().description = "Tutorial for getting familiar with the ORM."
task_tree = pycram.tasktree.task_tree
print(anytree.RenderTree(task_tree.root))
```

Next we serialize the task tree by recursively inserting from its root.

```python
task_tree.root.insert(session)
```

We can look at our experiment (Process)MetaData to get some context on the data we just created.

```python
from sqlalchemy import select

print(*session.scalars(select(pycram.orm.base.ProcessMetaData)).all())
```

Lastly we can look at various table to see how the structures got logged.
For example, we can get all the navigate actions that occurred.

```python
navigations = session.scalars(select(pycram.orm.action_designator.NavigateAction)).all()
print(*navigations, sep="\n")
```

Due to the inheritance mapped in the ORM package, we can also obtain all executed actions with just one query.

```python
actions = session.scalars(select(pycram.orm.action_designator.Action)).all()
print(*actions, sep="\n")
```

Of course all relational algebra operators, such as filtering and joining also work in pycram.orm queries. Let's say we need all the poses of objects, that were picked up by a robot. Since we defined a relationship between the PickUpAction table and the Object table and between the Object table and the Pose table in the ORM class schema, we can just use the join operator without any further specification:

```python
object_actions = (session.scalars(select(pycram.orm.base.PoseStamped)
                                  .join(pycram.orm.action_designator.PickUpAction.object)
                                  .join(pycram.orm.object_designator.Object.pose))
                  .all())
print(*object_actions, sep="\n")

```

Did you notice, that for the joins we did not join the tables together in a typical sql kind of way, but rather used the relationships defined in the ORM classes and wrote joins like PickUpAction.object or Object.pose? This is because the ORM package automatically creates the joins for us, so we only have to join on the attributes that hold the relationship. This is a huge advantage over writing sql queries by hand, since we do not have to worry about the join conditions. 
This is a strong tool, but it is crucial to use it properly. Very important to note: The order of the joins matters! For instance, if we joined the Pose table with the Object table first, and placed the join between the PickUpAction table and the Object table second, sqlalchemy would have selected the Pose not from the join between all three tables, but rather from a join between the Pose and the Object table + from a join between the PickUpAction table and the Object table. These mistakes can lead to wrong results or even to errors (the above-mentioned example would actually lead to an error due to the Object table being accessed twice in two separate joins in the same query and therefore the column names of the Object tables would have been ambiguous and could not be used by sqlalchemy to join).

Make sure to check out the other examples of ORM querying.


If we want to filter for all successful tasks we can just add the filter operator:

```python
from pycram.orm.tasktree import TaskTreeNode

successful_tasks = session.scalars(select(TaskTreeNode).where(TaskTreeNode.status == "SUCCEEDED"))
print(*successful_tasks, sep="\n")
```

As expected all but the root node succeeded, since the root node is still running.

Writing an extension to the ORM package is also done with ease. We need to create a new ActionDesignator class and its ORM equivalent, where we define our new table. Let's say we want to log all the things the robot says. We will create a new ActionDesignator class called Saying and its ORM equivalent called ORMSaying. 

```python
from sqlalchemy.orm import Mapped, mapped_column, Session
from pycram.orm.action_designator import Action
from dataclasses import dataclass


# define ORM class from pattern in every pycram.orm class
class ORMSaying(Action):

    id: Mapped[int] = mapped_column(sqlalchemy.ForeignKey(f'{Action.__tablename__}.id'), primary_key=True, init=False)
    # since we do not want to add any custom specifications to our column, we don't even need to define mapped_column, sqlalchemy does this internally.
    text: Mapped[str] 

# define brand new action designator
# Since this class is derived from ActionAbstract, we do not need to manually define the insert() and to_sql() function, the mapping is done automatically. We just have to tell the class, which ORMclass it is supposed to use.
@dataclass 
class SayingActionPerformable(ActionAbstract):
    
    text: str
    orm_class = ORMSaying
        
    @with_tree
    def plan(self) -> None:
        print(self.text)

    def validate(self, result=None):
        pass
```

Now we got our new ActionDesignator called Saying and its ORM version. Since this class got created after all other classes got inserted into the database (in the beginning of the notebook) we have to insert it manually. 

```python
ORMSaying.metadata.create_all(bind=engine)
```

Now we can create and insert a Saying action. Since this is the last part where we interact with the BulletWorld, we can also close it.

```python
# create a saying action and insert it
SayingActionPerformable("Patchie, Patchie; Where is my Patchie?").perform()
pycram.tasktree.task_tree.root.insert(session)
session.commit()

world.exit()
```

It is notable that committing the object to the session fills its primary key. Hence, there is no worries about assigning unique IDs manually.
Finally, we can double-check that our object exists in the database.

```python
session.scalars(select(ORMSaying)).all()
```
