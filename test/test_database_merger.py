import json
import traceback
import unittest
import pathlib

import sqlalchemy
import pycram.orm.base
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
import pycram.orm.utils

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.enums import Arms
from pycram.task import with_tree
import pycram.task
from pycram.bullet_world import BulletWorld, Object
from pycram.designators.object_designator import *
import anytree
from anytree import Node,RenderTree,LevelOrderIter

class ExamplePlans():
    def __init__(self):
        self.world = BulletWorld("DIRECT")
        self.pr2 = Object("pr2", "robot", "pr2.urdf")
        self.kitchen = Object("kitchen", "environment", "kitchen.urdf")
        self.milk = Object("milk", "milk", "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
        self.cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]),
                             color=[0, 1, 0, 1])
        self.milk_desig = ObjectDesignatorDescription(names=["milk"])
        self.cereal_desig = ObjectDesignatorDescription(names=["cereal"])
        self.robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
        self.kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    @with_tree
    def pick_and_place_plan(self):
        with simulated_robot:
            ParkArmsAction.Action(Arms.BOTH).perform()
            MoveTorsoAction([0.3]).resolve().perform()
            pickup_pose = CostmapLocation(target=self.cereal_desig.resolve(), reachable_for=self.robot_desig).resolve()
            pickup_arm = pickup_pose.reachable_arms[0]
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

            ParkArmsAction.Action(Arms.BOTH).perform()

    def clear_tree(self):  # not sure if needed
        pycram.task.reset_tree()


class MergerTestCaseBase(unittest.TestCase):
    def assertIsFile(self, in_path):
        if not pathlib.Path(in_path).resolve().is_file():
            raise AssertionError("Config File not found:{}".format(in_path))


# Note: Can't test full functionallity

class MergeDatabaseTest(unittest.TestCase):
    source_engine: sqlalchemy.engine.Engine
    destination_engine: sqlalchemy.engine.Engine
    source_session_maker: sqlalchemy.orm.sessionmaker
    destination_session_maker: sqlalchemy.orm.sessionmaker

    @unittest.skipIf(not (pathlib.Path("test_database_merger.json").resolve().is_file()),
                     "Config File not found: test_database_merger.json")
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        with open("test_database_merger.json") as f:
            config=json.load(f)
            if ("postgres" in config):
                connection_string="postgresql+psycopg2://{}:{}@{}:{}/{}".format(config["postgres"]["user"],config["postgres"]["password"],config["postgres"]["ipaddress"],config["postgres"]["port"],config["postgres"]["database"])
                cls.destination_engine=sqlalchemy.create_engine(connection_string,echo=False)
        cls.source_engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
        # cls.destination_engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
        cls.source_session_maker = sqlalchemy.orm.sessionmaker(bind=cls.source_engine)
        cls.destination_session_maker = sqlalchemy.orm.sessionmaker(bind=cls.destination_engine)
        source_session = cls.source_session_maker()
        destination_session = cls.destination_session_maker()
        pycram.orm.base.Base.metadata.create_all(cls.source_engine)
        pycram.orm.base.Base.metadata.create_all(cls.destination_engine)
        source_session.commit()
        destination_session.commit()
        source_session.close()
        destination_session.close()

    def setUp(self) -> None:
        super().setUp()
        source_session = self.source_session_maker()
        # destination_session = self.destination_session_maker()
        example_plans = ExamplePlans()
        for i in range(2):
            try:
                print("ExamplePlans run {}".format(i))
                example_plans.pick_and_place_plan()
                example_plans.world.reset_bullet_world()
                pycram.orm.base.MetaData().description = "Unittest: Example pick and place {}".format(i)
            except Exception as e:
                print("Error: {}\n{}".format(type(e).__name__, e))

        source_meta_data = pycram.orm.base.MetaData()
        source_meta_data.description = "Not all who wander are lost"
        source_meta_data.insert(source_session)
        pycram.task.task_tree.root.insert(source_session)
        source_meta_data.reset()
        # destination_metadata = pycram.orm.base.MetaData()
        # destination_metadata.description = "Not all that glitters is gold"
        # pycram.task.task_tree.root.insert(destination_session)
        # destination_metadata.insert(destination_session)
        # if (source_meta_data == destination_metadata):
        #     print("Then you are lost")
        source_session.commit()
        # destination_session.commit()
        example_plans.world.exit()
        source_session.close()
        # destination_session.close()

    def tearDown(self) -> None:
        super().tearDown()

    @classmethod
    def TearDownClass(cls):
        super().TearDownClass()

    def test_merge_databases(self):
        all_orm_classes_set = pycram.orm.utils.get_all_children_set(pycram.orm.base.Base)
        orm_tree = pycram.orm.utils.get_tree(pycram.orm.base.Base)
        ordered_orm_classes = [node.name for node in LevelOrderIter(orm_tree)]
        pycram.orm.utils.update_primary_key_constrains(self.destination_session_maker,ordered_orm_classes)
        pycram.orm.utils.update_primary_key(self.source_session_maker,
                                            self.destination_session_maker)  # self.source_engine,self.destination_engine)
        # pycram.orm.utils.print_database(self.destination_session_maker)
        # print("Source:" + 10 * '-')
        # pycram.orm.utils.print_database(self.source_session_maker)
        # pycram.orm.utils.copy_database(self.destination_session_maker,self.source_session_maker)
        pycram.orm.utils.copy_database(self.source_session_maker, self.destination_session_maker)
        pycram.orm.utils.print_database(self.source_session_maker)

# do stuff
