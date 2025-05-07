![](doc/images/pycram_logo.png)

# What is PyCRAM

PyCRAM is the Python 3 re-implementation of [CRAM](https://github.com/cram2/cram).
PyCRAM is a toolbox for designing, implementing and deploying software on autonomous robots. The framework provides various tools and libraries for aiding in robot software development as well as geometric reasoning and fast simulation mechanisms to develop cognition-enabled control programs that achieve high levels of robot autonomy.

PyCRAM is developed in Python with support for the ROS middleware which is used for communication with different software components as well as the robot.

This framework is tested with Ubuntu 20.04, ROS Noetic and Python 3.8

## Live Demo
**If you want to test out PyCRAM right away you can do that in the browser in our virtual research building [here](https://vib.ai.uni-bremen.de/page/fallschool/)**


## Simple Demonstartion
PyCRAM allows the execution of the same high-level plan on different robot platforms. Below you can see an example of this where the plan is executed on the PR2 and the IAIs Boxy.

|               Boxy                |          PR2            |
|:---------------------------------:|:-----------------------:|
| ![image alt](doc/images/boxy.gif) | ![](doc/images/pr2.gif) |

The plan that both robots execute is a relativly simple pick and place plan:
* They start at the world origin
* park their arms
* move to the counter
* observe the object
* pickup the object
* move to the kitchen island
* place the object
* move to the world origin

The code for this plan can be seen below.
```
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.process_module import simulated_robot
from pycram.designators.motion_designator import *
from pycram.designators.location_designator import *
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, Arms, Grasp, WorldMode
from pycram.language import SequentialPlan, ParallelPlan

world = BulletWorld(WorldMode.GUI)
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([1.4, 1, 0.95]))

cereal_desig = ObjectDesignatorDescription(names=["cereal"])
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

with simulated_robot:
    SequentialPlan(
        ParallelPlan(    
        ParkArmsAction([Arms.BOTH]),

        MoveTorsoAction([TorsoState.HIGH])),


    NavigateAction(target_locations=CostmapLocation(target=cereal_desig.resolve(), reachable_for=robot_desig).resolve()]),

    PickUpAction(object_designator_description=cereal_desig, arms=[Arms.LEFT], grasps=[Grasp.FRONT]),

    ParkArmsAction([Arms.BOTH]),

    NavigateAction(target_locations=CostmapLocation( 
        SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve(), cereal_desig.resolve()), 
        reachable_for=robot_desig, reachable_arm=Arms.LEFT)),

    PlaceAction(cereal_desig, target_locations=[place_island.pose], arms=[Arms.LEFT]),

    ParkArmsAction([Arms.BOTH]),
world.exit()
```


## Installation
For information on installing PyCRAM please check the guid [here](https://pycram.readthedocs.io/en/latest/installation.html).

## Documentation

The latest version of the documentation is hosted on Read the Docs [here](https://pycram.readthedocs.io/en/latest/index.html).

The documentation can be found in the `doc` folder, for instructions on how to build and view the documentation please 
take a look at the respective `README` file.

## Examples
Examples of features can be found either in the documentation under the 'Examples' Section or in the `examples` folder. 
The examples in the `examples` folder are Jupyter Notebooks which can be viewed and executed, for more information 
how to do that take a look at the respective `README` file. 

## Troubleshooting 
If you encounter some error please first take a look at the 
[troubleshooting](https://pycram.readthedocs.io/en/latest/troubleshooting.html) section and see if the error is mentioned 
there. 

# Virtual Building
Within our virtual building, you can find a variety of labs and examples that showcase the use of PyCRAM. These resources are available at our [Labs page](https://vib.ai.uni-bremen.de/page/labs/). They are designed to help you understand and experiment with PyCRAM's capabilities.

## Setting Up Your Own Lab

If you're looking to set up your own lab within the virtual building, please refer to the `vrb` branch of this repository. It includes detailed instructions and templates to guide you through the process.
