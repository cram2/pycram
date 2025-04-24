import unittest
import pathlib
import json
import sqlalchemy
# import pycram.orm.base
import pycram.orm.utils

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import Arms, ObjectType, WorldMode
from pycram.tasktree import with_tree
import pycram.tasktree
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.designators.object_designator import *
from dataclasses import dataclass, field


@dataclass
class Configuration:
    user: str = field(default="alice")
    password: str = field(default="alice123")
    ipaddress: str = field(default="localhost")
    port: int = field(default=5432)
    database: str = field(default="pycram")


class ExamplePlans:
    def __init__(self):
        self.world = BulletWorld(WorldMode.DIRECT)
        self.pr2 = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
        self.kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
        self.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=PoseStamped.from_list([1.3, 1, 0.9]))
        self.cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=PoseStamped.from_list([1.3, 0.7, 0.95]))
        self.milk_desig = ObjectDesignatorDescription(names=["milk"])
        self.cereal_desig = ObjectDesignatorDescription(names=["cereal"])
        self.robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
        self.kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    @with_tree
    def pick_and_place_plan(self):
        with simulated_robot:
            ParkArmsActionPerformable(Arms.BOTH).perform()
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

            ParkArmsActionPerformable(Arms.BOTH).perform()


class MergerTestCaseBase(unittest.TestCase):
    def assertIsFile(self, in_path):
        if not pathlib.Path(in_path).resolve().is_file():
            raise AssertionError("Config File not found:{}".format(in_path))


# Note: Can't test full functionality

class MergeDatabaseTest(unittest.TestCase):
    source_engine: sqlalchemy.engine.Engine
    destination_engine: sqlalchemy.engine.Engine
    source_session_maker: sqlalchemy.orm.sessionmaker
    destination_session_maker: sqlalchemy.orm.sessionmaker
    numbers_of_example_runs: int

    @unittest.skipIf(not (pathlib.Path("test_database_merger.json").resolve().is_file()),
                     "Config File not found: test_database_merger.json")
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        with open("test_database_merger.json") as f:
            json_data = json.load(f)
            config = Configuration(**json_data)
        connection_string = "postgresql+psycopg2://{}:{}@{}:{}/{}".format(config.user, config.password,
                                                                          config.ipaddress, config.port,
                                                                          config.database)
        cls.destination_engine = sqlalchemy.create_engine(connection_string, echo=False)
        cls.source_engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
        cls.source_session_maker = sqlalchemy.orm.sessionmaker(bind=cls.source_engine)
        cls.destination_session_maker = sqlalchemy.orm.sessionmaker(bind=cls.destination_engine)
        destination_session = cls.destination_session_maker()
        pycram.orm.base.Base.metadata.create_all(cls.destination_engine)
        destination_session.commit()
        destination_session.close()
        cls.numbers_of_example_runs = 3

    def setUp(self) -> None:
        super().setUp()
        source_session = self.source_session_maker()
        # If there is no session (connection to the memory database) it resets thus we need to define this here
        pycram.orm.base.Base.metadata.create_all(self.source_engine)
        example_plans = ExamplePlans()
        for i in range(self.numbers_of_example_runs):
            try:
                print("ExamplePlans run {}".format(i))
                example_plans.pick_and_place_plan()
                example_plans.world.reset_bullet_world()
                process_meta_data = pycram.orm.base.ProcessMetaData()
                process_meta_data.description = "Database merger Unittest: Example pick and place {}".format(i)
                process_meta_data.insert(source_session)
                pycram.task.task_tree.root.insert(source_session)
                process_meta_data.reset()
            except Exception as e:
                print("Error: {}\n{}".format(type(e).__name__, e))
        source_session.commit()
        example_plans.world.exit()
        source_session.close()

    def tearDown(self) -> None:
        super().tearDown()

    @classmethod
    def TearDownClass(cls):
        super().TearDownClass()

    def test_merge_databases(self):
        pycram.orm.utils.update_primary_key_constrains(self.destination_session_maker)
        pycram.orm.utils.update_primary_key(self.source_session_maker, self.destination_session_maker)
        pycram.orm.utils.copy_database(self.source_session_maker, self.destination_session_maker)
        destination_content = dict()
        source_content = dict()
        with self.destination_session_maker() as session:
            for table in pycram.orm.base.Base.metadata.sorted_tables:
                table_content_set = set()
                table_content_set.update(session.query(table).all())
                destination_content[table] = table_content_set

        with self.source_session_maker() as session:
            for table in pycram.orm.base.Base.metadata.sorted_tables:
                table_content_set = set()
                table_content_set.update(session.query(table).all())
                source_content[table] = table_content_set

        for key in destination_content:
            self.assertEqual(destination_content[key], destination_content[key].union(source_content[key]))

    def test_migrate_neems(self):
        pycram.orm.utils.migrate_neems(self.source_session_maker, self.destination_session_maker)
        destination_content = dict()
        source_content = dict()
        with self.destination_session_maker() as session:
            for table in pycram.orm.base.Base.metadata.sorted_tables:
                table_content_set = set()
                table_content_set.update(session.query(table).all())
                destination_content[table] = table_content_set

        with self.source_session_maker() as session:
            for table in pycram.orm.base.Base.metadata.sorted_tables:
                table_content_set = set()
                table_content_set.update(session.query(table).all())
                source_content[table] = table_content_set

        for key in destination_content:
            self.assertEqual(destination_content[key], destination_content[key].union(source_content[key]))
