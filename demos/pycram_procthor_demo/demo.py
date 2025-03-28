import rospy

from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import PoseStamped

from pycram.datastructures.dataclasses import Color
from demos.pycram_procthor_demo.code_example import generic_plan
from pycram import World
from pycram.failures import PlanFailure
from pycram.tasktree import task_tree

import traceback
import sqlalchemy
import pycram.orm.base
from pycram.external_interfaces.procthor import ProcThorInterface
import pycrap

engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
session_maker = sqlalchemy.orm.sessionmaker(bind=engine)
session = session_maker()
# ToDo: Many Plans missing, but this needs function pointers and I am not comfortable with it
# plans=[]
robots = []
robot_name = "pr2.urdf"
robots.append(robot_name)
pycram.orm.base.Base.metadata.create_all(engine)
session.commit()
# Set up for ProcThor stuff Example usage
procThorInterface = ProcThorInterface(base_url="http://procthor.informatik.uni-bremen.de:5000/")
number_of_test_environment = 5

# Get Environments
number_of_known_environment = len(
    procThorInterface.get_all_environments_stored_below_directory(procThorInterface.source_folder))
print("Number of known Environments:{}".format(number_of_known_environment))
print("Number of needed Testenvironments:{}".format(number_of_test_environment))

if number_of_known_environment < number_of_test_environment:
    print("Downloading missing environments...")
    procThorInterface.download_num_random_environment(number_of_test_environment - number_of_known_environment)

def runWorld():
    counter = 0
    works = 0
    fails = 0
    known_environments = procThorInterface.get_all_environments_stored_below_directory(procThorInterface.source_folder)
    world = BulletWorld(WorldMode.GUI)
    apartment = None
    milk_pos = PoseSteamped.from_list([1, -1.78, 0.55], [1, 0, 0, 0])
    milk = Object("milk", pycrap.Milk, "milk.stl", pose=PoseSteamped.from_list([1, -1.78, 0.55], [1, 0, 0, 0]),
                  color=Color(1, 0, 0, 1))
    for robot in robots:
        robot_obj = Object("pr2", pycrap.Robot, robot, pose=PoseSteamped.from_list([1, 2, 0]))
        for environment in known_environments:
            counter += 1
            print("Trying plan: {} with robot: {} in: {}".format("param_plan", robot, environment["name"]))
            try:
                apartment = Object(environment["name"], pycrap.Apartment, environment["storage_place"]) # Is appartment correct?
                generic_plan(world)
                works += 1
                print("Successfully!\n Overall successful tries: {}".format(works))

            except PlanFailure as e:
                traceback.print_exc()
                fails += 1
                print("Plan Fail!\n Overall failed tries: {}".format(fails))

            except FileNotFoundError as e2:
                traceback.print_exc()
                fails += 1
                print("Fail!\n Overall failed tries: {}".format(fails))

            finally:
                try:
                    process_meta_data = pycram.orm.base.ProcessMetaData()
                    process_meta_data.description = "CEO Test run number {} robot:{} enviroment:{}".format(counter,
                                                                                                           robot,
                                                                                                           environment)
                    process_meta_data.insert(session)
                    task_tree.root.insert(session)
                except Exception as e:
                    traceback.print_exc()
                    print("Error while storing the NEEM. This should not happen.")
                task_tree.reset_tree()
                if World.current_world is not None and apartment is not None:
                    world.remove_object(apartment)
                    milk.set_pose(milk_pos)
                    print("reseting world")
    if World.current_world is None:
        world.remove_object(robot_obj)

    print("Resume:\nOverall successful tries: {}\nOverall failed tries: {}".format(works, fails))
    World.current_world.exit()


runWorld()