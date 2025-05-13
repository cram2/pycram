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
```

Next, we need a mapper_registry to map our classes to the database tables. We will use the default mapper_registry from sqlalchemy.

```python
import pycram.orm.ormatic_interface
from pycram.orm.ormatic_interface import *

pycram.orm.ormatic_interface.metadata.create_all(engine)
```

Next, we will write a simple plan where the robot parks its arms, moves somewhere, picks up an object, navigates somewhere else, and places it. 

```python
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import Arms, ObjectType, Grasp, WorldMode, TorsoState
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.object_designator import *
from pycram.datastructures.pose import PoseStamped
from pycrap.ontologies import Robot, Kitchen, Milk, Cereal
from pycram.language import SequentialPlan

world = BulletWorld(WorldMode.DIRECT)
pr2 = Object("pr2", Robot, "pr2.urdf")
kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([1.3, 1, 0.9]))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl", pose=PoseStamped.from_list([1.3, 0.7, 0.95]))
milk_desig = ObjectDesignatorDescription(names=["milk"])
cereal_desig = ObjectDesignatorDescription(names=["cereal"])
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

object_description = ObjectDesignatorDescription(names=["milk"])
with simulated_robot:
    sp = SequentialPlan(
        NavigateActionDescription(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True),
        ParkArmsActionDescription(Arms.BOTH),
        PickUpActionDescription(object_description.resolve(), Arms.LEFT, GraspDescription(Grasp.FRONT, None, False)),
        NavigateActionDescription(PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1]), True),
        PlaceActionDescription(object_description.resolve(), PoseStamped.from_list([2.0, 1.6, 1.8], [0, 0, 0, 1]),
                               Arms.LEFT))
    sp.perform()
```

The data obtained throughout the plan execution, including robot states, poses, action descriptions and more will be
logged into the database once we insert the plan .

```python
from pycram.orm.logging_hooks import insert

insert(sp, session)
```

Now we can query the database to see what we have logged. Let's say we want to see all the NavigateActions that occurred.

```python
from sqlalchemy import select
from pycram.designators.action_designator import NavigateAction

navigations = session.scalars(select(NavigateAction)).all()
print(*navigations, sep="\n")
```

This should print all the pick up actions that occurred during the plan execution, which is one.

Due to the inheritance mapped in the ORM package, we can also get all executed actions with just one query.

```python
from pycram.designator import ActionDescription

actions = session.scalars(select(ActionDescription)).all()
print(*actions, sep="\n")
```

This should print all the actions that occurred during the plan execution, which is five.

Of course, all relational algebra operations, such as filtering and joining also work in pycram.orm queries. 
Let's say we want all the positions of objects, that were picked up by a robot. This can be done via joins.

Sqlalchemy provides a very nice way to do join different tables by using so-called relationship patterns where an 
attribute of a class is defined as a relationship to another class. Every relationship references a ForeignKey attribute,
which points to the other class. Relationship attributes can be used to join the tables together. 

In practice, the join would look like this:

```python
from pycram.datastructures.pose import Vector3, Pose, PoseStamped
from pycram.datastructures.dataclasses import FrozenObject
from pycram.designators.action_designator import PickUpAction

object_actions = (session.scalars(select(Vector3)
                                      .join(PickUpAction.object_at_execution)
                                      .join(FrozenObject.pose)
                                      .join(PoseStamped.pose)
                                      .join(Pose.position)).all())
print(*object_actions, sep="\n")
```

Did you notice, that for the joins we did not join the tables together in a typical sql kind of way, 
but rather used the relationship attributes? This is because the ORM package automatically creates the joins for us, 
so we only have to join on the attributes that hold the relationship. 
This is a huge advantage over writing sql queries by hand, since we do not have to worry about the join conditions. 
This is a strong tool, but it is crucial to use it properly. 
Very important to note: The order of the joins matters! 
For instance, if we joined the Pose table with the FrozenObject table first, and placed the join between 
the PickUpAction table and the FrozenObject table second, sqlalchemy would have selected the Pose not from the join 
between all three tables, but rather from a join between the Pose and the FrozenObject table + from a join between 
the PickUpAction table and the Object table. 
These mistakes can lead to wrong results or even to errors (the above-mentioned example would actually lead to an error 
due to the FrozenObject table being accessed twice in two separate joins in the same query and therefore the column 
names of the FrozenObject tables would have been ambiguous and could not be used by sqlalchemy to join).

Make sure to check out the other examples of ORM querying.


If we want to filter for all successful tasks we can just add the filter operator:

```python
filtered_navigate_results = session.scalars(select(NavigateAction).where(NavigateAction.id == 1)).all()
print(*filtered_navigate_results, sep="\n")
```

As expected, only the first navigate action is printed. 


## Extending the ORM
Now we know how to work with the ORM. How can we *extend* it with new classes though?

Writing an extension to the ORM package is also done with ease. 
Generally speaking, we differentiate between two types of extensions:
1. Let's say we added a new action designator to pycram and we want to log the whole action to the database. 
In this case we just have to add the new designator to the list called **self_mapped_classes** in pycram.orm.model.
2. Let's say we want to add a new action designator to pycram, but we only want to log a part of the action 
(some attributes) to the database.
In this case we have to create a new ORM class and add its type to **explicitly_mapped_classes**, also in the pycram.orm.model.
The new class has to follow this pattern:

```python
from dataclasses import dataclass
from ormatic.utils import ORMaticExplicitMapping, classproperty
from pycram.datastructures.enums import Arms
from pycram.datastructures.grasp import GraspDescription
from pycram.designators.action_designator import PickUpAction as PickUpActionDesignator
from typing_extensions import Optional
from pycram.datastructures.dataclasses import FrozenObject


# create new dataclass, it has to inherit from ORMaticExplicitMapping
@dataclass
class PickUpAction(ActionDescription, ORMaticExplicitMapping):
  # add all attributes that we want to log to the database (must be the same names as in the action designator)
  arm: Arms
  prepose_distance: float
  grasp_description: GraspDescription
  object_at_execution: Optional[FrozenObject]

  # the explicit_mapping has to return the class of the action designator we want to map
  @classproperty
  def explicit_mapping(cls):
    return PickUpActionDesignator
```

Once we have updated the orm structure, we have to build the ORM package again.
This can be done by running the generate_orm.py script in pycram.scripts.

### Remarks

Adding classes to the orm assumes that all the types of the attributes of the new action designator are already mapped in the ORM package
or resemble a builtin type.
If this is not the case, we have two options:
1. We also have to add the type of the attribute to the orm as described above or
2. We cast the logging of the new type to a builtin type within the database.
If you want to store the custom type as a string in the database, you can map the type to the StringType() in the type_mappings dict in pycram.orm.model.

