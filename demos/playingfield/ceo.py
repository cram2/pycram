from pycram import World
from pycram.plan_failures import PlanFailure
from pycram.tasktree import task_tree, reset_tree
from sandcastle import param_plan
import traceback
import hashlib
import sqlalchemy
import pycram.orm.base

# extension = ObjectDescription.get_file_extension()
# Many Plans in many environments with many robots
# Don't know where to store NEEM so I start with the memory db
# engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
# session_maker = sqlalchemy.orm.sessionmaker(bind=engine)
# session = session_maker()
# ToDo: Many Plans missing, but this needs function pointers and I am not comfortable with it
# plans=[]
robots =[]
environments=[]
robot_name="pr2.urdf"
environment_name="house_2.urdf"
robots.append(robot_name)
environments.append(environment_name)
environments.append("Murphys_Law")
environments.append("apartment.urdf")
# pycram.orm.base.Base.metadata.create_all(engine)
# session.commit()


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
        finally:
            World.current_world.exit()

print("Resume:\nOverall successful tries: {}\nOverall failed tries: {}".format(works,fails))