import anytree
import psutil
import rospy
from txaio.tx import sleep
import os
import requests
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import Pose

from pycram.datastructures.dataclasses import Color
from demos.playingfield.sandcastle import generic_plan
from pycram import World
from pycram.external_interfaces.giskard import giskard_wrapper
from pycram.plan_failures import PlanFailure, IKError
from pycram.tasktree import task_tree, reset_tree
from sandcastle import param_plan,param_plan2
import traceback
import hashlib
import sqlalchemy
import pycram.orm.base
import pycram.external_interfaces.giskard as giskard
import requester

# extension = ObjectDescription.get_file_extension()
# Many Plans in many environments with many robots
# Don't know where to store NEEM so I start with the memory db
engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
session_maker = sqlalchemy.orm.sessionmaker(bind=engine)
session = session_maker()
# ToDo: Many Plans missing, but this needs function pointers and I am not comfortable with it
# plans=[]
robots =[]
environments=[]
robot_name="pr2.urdf"
environment_name="house_2.urdf"
robots.append(robot_name)
# environments.append("house_42.urdf")g
environments.append("apartment.urdf")
environments.append("house_1337.urdf")
environments.append("house_666.urdf")
# environments.append(environment_name)
# environments.append("Murphys_Law")
pycram.orm.base.Base.metadata.create_all(engine)
session.commit()
# Example usage
base_url = 'http://192.168.102.44/environment/'
download_folder = "/home/nleusmann/catkin_ws/src/pycram/resources/tmp/"
if len(os.listdir(download_folder)) == 0:
    print("Downloading new environments")
    requester.download_all_files(base_url, download_folder)
else:
    print("Environment folder not empty. Stopped download")


def run():
    v=0
    works=0
    fails=0
    for robot in robots:
        for environment in requester.get_all_environments(download_folder):
            v+=1
            print("Trying plan: {} with robot: {} in: {}".format("param_plan", robot, environment))
            try:
                param_plan2(robot, environment)
                works+=1
                print("Successfully!\n Overall successful tries: {}".format(works))

            except PlanFailure as e:
                traceback.print_exc()
                fails+=1
                print("Plan Fail!\n Overall failed tries: {}".format(fails))

            except rospy.service.ServiceException as service_error:
                traceback.print_exc()
                fails+=1
                print("Service call failed!\n Overall failed tries: {}".format(fails))

            # except pycram.plan_failures.IKError as ik_error:
            #     traceback.print_exc()
            #     fails+=1
            #     print("IK Fail!\n Overall failed tries: {}".format(fails))
            except FileNotFoundError as e2:
                traceback.print_exc()
                fails += 1
                print("Fail!\n Overall failed tries: {}".format(fails))

            finally:
                try:
                    process_meta_data = pycram.orm.base.ProcessMetaData()
                    process_meta_data.description = "CEO Test run number {} robot:{} enviroment:{}".format(v,robot,environment)
                    process_meta_data.insert(session)
                    task_tree.root.insert(session)
                except Exception as e:
                    traceback.print_exc()
                    print("Error while storing the NEEM. This should not happen.")
                reset_tree()
                if World.current_world is not None:
                    World.current_world.exit()
                # sleep(2)
                # giskard.giskard_wrapper.clear_world()

                #clear giskard world?

    print("Resume:\nOverall successful tries: {}\nOverall failed tries: {}".format(works,fails))


def runWorld():
    v = 0
    works = 0
    fails = 0
    world=BulletWorld(WorldMode.GUI)
    apartment=None
    milk_pos = Pose([1, -1.78, 0.55], [1, 0, 0, 0])
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1, -1.78, 0.55], [1, 0, 0, 0]),
                  color=Color(1, 0, 0, 1))
    for robot in robots:
        robot_obj=Object("pr2", ObjectType.ROBOT, robot, pose=Pose([1, 2, 0]))
        for environment in requester.get_all_environments(download_folder):
            v += 1
            print("Trying plan: {} with robot: {} in: {}".format("param_plan", robot, environment))
            try:
                house_name=environment.split("/")[-1].split(".")[0]
                apartment = Object(house_name, ObjectType.ENVIRONMENT,environment)
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
                    process_meta_data.description = "CEO Test run number {} robot:{} enviroment:{}".format(v, robot,
                                                                                                           environment)
                    process_meta_data.insert(session)
                    task_tree.root.insert(session)
                except Exception as e:
                    traceback.print_exc()
                    print("Error while storing the NEEM. This should not happen.")
                reset_tree()
                if World.current_world is not None and apartment is not None:
                    world.remove_object(apartment)
                    milk.set_pose(milk_pos)
                    print("reseting world")
    if World.current_world is None:
        world.remove_object(robot_obj)


    print("Resume:\nOverall successful tries: {}\nOverall failed tries: {}".format(works, fails))
    World.current_world.exit()
runWorld()