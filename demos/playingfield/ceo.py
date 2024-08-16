from pycram import World
from sandcastle import param_plan
import traceback
import hashlib
# extension = ObjectDescription.get_file_extension()
# Many Plans in many environments with many robots
# ToDo: Many Plans missing, but this needs function pointers and I am not comfortable with it
# plans=[]
robots =[]
environments=[]
robot_name="pr2.urdf"
environment_name="house_2.urdf"
robots.append(robot_name)
environments.append(environment_name)
environments.append("apartment.urdf")
environments.append("Murphys_Law")


works=0
fails=0
for robot in robots:
    for environment in environments:
        print("Trying plan: {} with robot: {} in: {}".format("param_plan", robot, environment))
        try:
            param_plan(robot, environment)
            works+=1
            print("Successfully!\n Overall successful tries: {}".format(works))

        except Exception as e:
            traceback.print_exc()
            hash(e) # Random Error Value (for now)
            fails+=1
            print("Fail!\n Overall failed tries: {}".format(fails))
        finally:
            World.current_world.exit()

print("Resume:\nOverall successful tries: {}\nOverall failed tries: {}".format(works,fails))