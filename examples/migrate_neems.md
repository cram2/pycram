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

# Migrate NEEMs

In this tutorial we will go through the process of migrating locally stored PyCRORM NEEMs to an already existing 
PyCRORM NEEM-Hub.

In some cases it my occur that you want to record data from a pycram controlled robot locally and perform some local 
actions before migrating your data to a big database server. In such cases, you can easily make a local database and
connect your pycram process to it. 

After you recorded your data locally you can migrate the data using the `migrate_neems` function.

First, lets create an in-memory database engine called `source_engine` where we record our current process.

```python
import sqlalchemy.orm
import pycram

source_engine: sqlalchemy.engine.Engine
source_engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
source_session_maker = sqlalchemy.orm.sessionmaker(bind=source_engine)
pycram.orm.base.Base.metadata.create_all(source_engine) #create all Tables
```

Next, create an engine called `destination_engine` for the destination database where you want to migrate your NEEMs to.
`Note:` This is just an example configuration.

```python
destination_engine: sqlalchemy.engine.Engine
destination_engine = sqlalchemy.create_engine("postgresql+psycopg2://alice:alice123@localhost:5433/pycram", echo=False) # example values
destination_session_maker = sqlalchemy.orm.sessionmaker(bind=destination_engine)
```

If you already have some data in your local database you can skip the next block, otherwise we will quickly create 
some example data

```python
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.tasktree import with_tree
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.designators.object_designator import *
from pycrap.ontologies import Robot, Kitchen, Milk, Cereal


class ExamplePlans:
    def __init__(self):
        self.world = BulletWorld("DIRECT")
        self.pr2 = Object("pr2", Robot, "pr2.urdf")
        self.kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
        self.milk = Object("milk", Milk, "milk.stl", pose=PoseSteamped.from_list([1.3, 1, 0.9]))
        self.cereal = Object("cereal", Cereal, "breakfast_cereal.stl", pose=PoseSteamped.from_list([1.3, 0.7, 0.95]))
        self.milk_desig = ObjectDesignatorDescription(names=["milk"])
        self.cereal_desig = ObjectDesignatorDescription(names=["cereal"])
        self.robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
        self.kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    @with_tree
    def pick_and_place_plan(self):
        with simulated_robot:
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            MoveTorsoAction([0.3]).resolve().perform()
            pickup_pose = CostmapLocation(target=self.cereal_desig.resolve(), reachable_for=self.robot_desig).resolve()
            pickup_arm = pickup_pose.reachable_arm
            NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()
            PickUpAction(object_designator_description=self.cereal_desig, arms=[pickup_arm],
                         grasps=["front"]).resolve().perform()
            ParkArmsAction([Arms.BOTH]).resolve().perform()

            place_island = SemanticCostmapLocation("kitchen_island_surface", self.kitchen_desig.resolve(),
                                                   self.cereal_desig.resolve()).resolve()

            place_stand = CostmapLocation(place_island.pose, reachable_for=self.robot_desig,
                                          reachable_arm=pickup_arm).resolve()

            NavigateAction(target_locations=[place_stand.pose]).resolve().perform()

            PlaceAction(self.cereal_desig, target_locations=[place_island.pose], arms=[pickup_arm]).resolve().perform()

            ParkArmsAction([Arms.BOTH]).resolve().perform()

```

```python
import pycram.orm.utils           
import pycram.tasktree
            
with source_session_maker() as session:
    example_plans = ExamplePlans()
    for i in range(3):
        try:
            print("ExamplePlans run {}".format(i))
            example_plans.pick_and_place_plan()
            example_plans.world.reset_bullet_world()
            process_meta_data = pycram.orm.base.ProcessMetaData()
            process_meta_data.description = "Example Plan {}".format(i)
            process_meta_data.insert(session)
            pycram.tasktree.task_tree.root.insert(session)
            process_meta_data.reset()
        except Exception as e:
            print("Error: {}\n{}".format(type(e).__name__, e))
    session.commit()
    example_plans.world.exit()
```

Now that we have some example data or already had some example data all we need to do it migrate it over to
the already existing PyCRORM NEEM-Hub.

```python
pycram.orm.utils.migrate_neems(source_session_maker,destination_session_maker)
```

If the command ran successful the content of the source database should now be copied within the destination database. For example if we query for all the different meta_data, the previously defined instance come up.

```python
with destination_session_maker() as session:
    statement = sqlalchemy.select('*').select_from(pycram.orm.base.ProcessMetaData)
    result = session.execute(statement).all()
    for item in result:
        print(item)
```

Looking at all the output, we can clearly see that the PyCRORM NEEM-Hub now contains our Example Plans 0 - 2. 
