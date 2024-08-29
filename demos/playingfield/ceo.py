import anytree
from txaio.tx import sleep

import requests
from pycram import World
from pycram.external_interfaces.giskard import giskard_wrapper
from pycram.plan_failures import PlanFailure
from pycram.tasktree import task_tree, reset_tree
from sandcastle import param_plan
import traceback
import hashlib
import sqlalchemy
import pycram.orm.base
import pycram.external_interfaces.giskard as giskard

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

server_ip="192.168.102.44"
server_path="/environment"
v=0
works=0
fails=0
for robot in robots:
    for environment in environments:
        v+=1
        print("Trying plan: {} with robot: {} in: {}".format("param_plan", robot, environment))
        try:
            param_plan(robot, environment)
            works+=1
            print("Successfully!\n Overall successful tries: {}".format(works))

        except PlanFailure as e:
            traceback.print_exc()
            fails+=1
            print("Fail!\n Overall failed tries: {}".format(fails))
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
            World.current_world.exit()
            # giskard.giskard_wrapper.clear_world()

            #clear giskard world?

print("Resume:\nOverall successful tries: {}\nOverall failed tries: {}".format(works,fails))