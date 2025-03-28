---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.3
  kernelspec:
    display_name: Python 3 (ipykernel)
    language: python
    name: python3
---

# How to create Knowledge Source
(knowledge_source_header)=
This notebook will detail what a knowledge source does, how it works and how you can create your own. 

A knowledge source is part of the wider knowledge system in PyCRAM as explained [here](/knowledge). The purpose of a 
knowledge source is to provide an interface to an external knowledge and reasoning system.

A knowledge source essentially consists of two parts, management methods which take care of connecting to the knowledge 
system as well as registering the knowledge source with the knowledge engine and the implementation of the respective 
reasoning queries which this knowledge source is able to process. 

In this example we will walk through the process of creating a simple knowledge source and all steps involved in this process. 

## Creating the Knowledge Source structure

We will start by creating the general structure of the Knowledge Source as well as the management methods. To do this 
you have to create a new class which inherits from the ```KnowledgeSource``` class.

```python
from pycram.knowledge.knowledge_source import KnowledgeSource

class ExampleKnowledge(KnowledgeSource):
    
    def __init__(self):
        super().__init__(name="example", priority=0)
        self.parameter = {}
        
    def is_available(self) -> bool:
        return True
    
    def is_connected(self) -> bool:
        return True
    
    def connect(self):
        pass
    
    def clear_state(self):
        self.parameter = {}
```

What we did in the code above was creating a class which inherits from the ```KowledgeSource``` base class, in the 
constructor of this new class we initialize the base class with the name of the new Knowledge Source as well as a 
priority. The priority is used to determine the order of all Knowledge Sources, in case two Knowledge Sources provide 
the same reasoning queries the one with the higher priority is used. 

Furthermore, we define a number of methods that manage the connection to the knowledge system namely the methods 
```is_available```, ```is_connected``` and ```connect```. The first two methods just return a bool which states if the 
knowledge system is available and connected to this Knowledge Source. The last method is used to create a connection 
to the knowledge system. Since this is an example and we are not connecting to an external knowledge system the methods 
are fairly trivial. 

The last method we defined is ```clear_state```, this is used to clear any state the knowledge source itself might hold 
either of the knowledge system or of this Knowledge Source class itself. 


# Managing the resolvable Properties 
Properties serve two purposes in the management of knowledge in PyCRAM, the first is to define semantic properties of 
parameter of action designator. The second is to specify which properties or knowledge queries a Knowledge Source can 
answer. 

To define which properties a Knowledge Source can handle we simply use the respective properties as mix-in for the class 
definition. With this let's take another look at our Knowledge Source with the handling of two properties.

```python
from pycram.knowledge.knowledge_source import KnowledgeSource
from pycram.datastructures.property import ReachableProperty, SpaceIsFreeProperty
from pycram.datastructures.world import World
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.dataclasses import ReasoningResult
from pycram.costmaps import OccupancyCostmap
from pycram.ros import loginfo
import numpy as np


class ExampleKnowledge(KnowledgeSource, ReachableProperty, SpaceIsFreeProperty):

    def __init__(self):
        super().__init__(name="example", priority=0)
        self.parameter = {}

    def is_available(self) -> bool:
        return True

    def is_connected(self) -> bool:
        return True

    def connect(self):
        pass

    def clear_state(self):
        self.parameter = {}

    def reachable(self, pose: PoseStamped) -> ReasoningResult:
        loginfo(f"Checking reachability for pose {pose}")
        robot_pose = World.robot.pose
        distance = pose.dist(robot_pose)
        return ReasoningResult(distance < 0.6)

    def space_is_free(self, pose: PoseStamped) -> ReasoningResult:
        loginfo(f"Checking if the space is free around {pose}")
        om = OccupancyCostmap(0.2, False, 100, 0.02, pose)
        return ReasoningResult(np.sum(om.map) == 6561)
```

Now we extend our Knowledge Source with the capability to handle two properties, Reachable and SpaceIsFree. As you can 
see all we needed to do for this is to use the respective properties as mix-ins besides the Knowledge Source as well as 
implement the method for each property which does the actual reasoning. 

In this case the reasoning is kept fairly simple, since this is not the objective of this example. Reachable just 
checks if a pose is within 60 centimeters of the robot while SpaceIsFree checks if a 2x2 meter square around the given 
pose has no obstacles. 

The methods doing the reasoning have to return a ```ReasoningResult``` instance, which contains a bool stating if the 
reasoning succeeded or failed as well as additional parameters which might be inferred during reasoning. The additional 
parameters are stated as key-value pairs in a dictionary.


# Testing the Knowledge Source
Since we now have a Knowledge Source which also implements two properties we can check if the Knowledge Source is used 
to resolve the correct properties. 

For this test we need a world as well as a robot.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import WorldMode, ObjectType
from pycram.knowledge.knowledge_engine import KnowledgeEngine
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.property import ReachableProperty, SpaceIsFreeProperty
from pycrap.ontologies import Robot

world = BulletWorld(WorldMode.GUI)
pr2 = Object("pr2", Robot, "pr2.urdf")

target_pose = PoseSteamped.from_list([0.3, 0, 0.2])
property = ReachableProperty(target_pose) & SpaceIsFreeProperty(target_pose)

ke = KnowledgeEngine()
resolved_property = ke.resolve_properties(property)

print(f"Result of the property: {resolved_property()}")
```

As you can see we created a ```ReachableProperty``` as well as a ```SpaceIsFreeProperty``` and resolved them. For more 
details on how properties and their resolution work please referee to the properties example. 

Afterwards, we execute the properties, here we can see the logging infos from our Knowledge Source as well as the 
confirmation that the implementation for both properties worked correctly.

```python
world.exit()
```
