# Procthor Interface - Creating a Manager for testing multible robots in mutliple enviromnents

This Notebook aims to provide an overview of the ProcThor Interface, by giving an easy-to-understand example on how to 
use it. We will be executing a simple pick and place plan with hard coded values and store the NEEMs of these 
experiments within a local memory database. 

Please be aware that this Notebook (currently) works only when you are connected to the network at the university in 
Bremen, otherwise we will not be able to connect to the REST service and the webserver itself.

If you want to check if you are able to run the Notebook simply try to ping the following server.

```batch

ping procthor.informatik.uni-bremen.de

```

If you get a positive answer back from the server everything seems to be all right. 

Now we need to set up the local memory database.
```python
import sqlalchemy
import pycram.orm.base

engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
session_maker = sqlalchemy.orm.sessionmaker(bind=engine)
session = session_maker()
pycram.orm.base.Base.metadata.create_all(engine)
session.commit()
```
Time for some more setup, while we can technically use multiple robots  in the different environments we for now stick 
with only PR2. Afterwards, we will start to set up our ProcThorInterface. Our example will work with 5 different 
environments.
```python
from pycram.external_interfaces.procthor import ProcThorInterface
from pycram.ros import ros_tools
pycram_path=ros_tools.get_ros_package_path('pycram')

robots =[]
robot_name="pr2.urdf"
robots.append(robot_name)


procthor_rest_endpoint = "http://procthor.informatik.uni-bremen.de:5000/"
source_folder = pycram_path + "resources/tmp/"
procThorInterface=ProcThorInterface(base_url=procthor_rest_endpoint,source_folder=source_folder)
number_of_test_environment=5
```
First let us check how many different environments are already known.

```python
number_of_known_environment = len(
    procThorInterface.get_all_environments_stored_below_directory(procThorInterface.source_folder))
log.loginfo("Number of known Environments:{}".format(number_of_known_environment))
log.loginfo("Number of needed Testenvironments:{}".format(number_of_test_environment))
```

Now depending on if we have to get some more, we will download some additional:
```python
if number_of_known_environment < number_of_test_environment:
    log.loginfo("Downloading missing environments...")
    procThorInterface.download_num_random_environment(number_of_test_environment - number_of_known_environment)
```

Now that the preparations are met we can start to run our plan for each robot (even though it is only one) and each
environment we know:

```python
from pycram.external_interfaces.pycramgym import PyCRAMGym  
    counter = 0
    works = 0
    fails = 0
    known_environments = procThorInterface.get_all_environments_stored_below_directory(procThorInterface.source_folder)
    world = BulletWorld(WorldMode.GUI) # If we spawn and despawn the bulletworld in quick sucession we will have missing object references
    milk_pos = Pose([1, -1.78, 0.55], [1, 0, 0, 0])
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1, -1.78, 0.55], [1, 0, 0, 0]),
                  color=Color(1, 0, 0, 1))
    for robot in robots:
        for environment in known_environments:
            counter += 1
            log.loginfo("Trying plan: {} with robot: {} in: {}".format("param_plan", robot, environment["name"]))
            try:
                # apartment = Object(environment["name"], ObjectType.ENVIRONMENT, environment["storage_place"])
                setupNr1=PyCRAMGym(environment["name"], robot, Code(language_plan_test))

                setupNr1.plan.perform()
                works += 1
                log.loginfo("Successfully!\n Overall successful tries: {}".format(works))

            except PlanFailure as e:
                traceback.print_exc()
                fails += 1
                log.loginfo("Plan Fail!\n Overall failed tries: {}".format(fails))

            except FileNotFoundError as e2:
                traceback.print_exc()
                fails += 1
                log.loginfo("Fail!\n Overall failed tries: {}".format(fails))

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
                    log.logerr("Error while storing the NEEM. This should not happen.")
                task_tree.reset_tree()
                if World.current_world is not None:
                    setupNr1.close()
                    log.loginfo("reseting world")
    # if World.current_world is None:
    #     world.remove_object(robot_obj)

    log.loginfo("Resume:\nOverall successful tries: {}\nOverall failed tries: {}".format(works, fails))
    World.current_world.exit()
```


