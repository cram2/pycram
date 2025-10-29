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
# Introduction to Plans
Plans in PyCRAM refer to a sequence of actions that are executed by the robot. Plans are created using the language 
expressions introduced in the [Language](language.md) section. Plans can be executed in a simulated environment or on a 
real robot. 

A plan consists of nodes these are either LanguageNodes which shape the control flow of the plan or DesignatorNodes 
are associated with a designator and can be performed by the robot.

We will now go through a simple example of how to create a plan using the PyCRAM language. To create a plan you always 
need a language expression.

# Setup a World 

```python
from pycram.process_module import simulated_robot
from pycram.testing import setup_world
from semantic_digital_twin.robots.pr2 import PR2

world = setup_world()

pr2 = PR2.from_world(world)
```


## Example Plan

```python
from pycram.robot_plans import *
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan

navigate = NavigateActionDescription(PoseStamped.from_list([1, 1, 0]))
park = ParkArmsActionDescription([Arms.BOTH])

plan = SequentialPlan((world, None), pr2, navigate, park)
```

This will create a simple plan which has a SequentialNode as its root and two DesignatorNodes as its children. You can 
plot the plan using the `plot_plan_structure` method.

```python
plan.plot_plan_structure()
```

## Arguments of Nodes

Nodes hava a number of arguments that provide information about the designator associated with the node and the current 
state of execution. Arguments of nodes include:

* status: The current status of the node including CREATED, RUNNING, SUCCEEDED, FAILED
* start_time/end_time: The time when the node started and ended execution
* reason: The reason for the failure during execution
* plan: A reference to the plan this node belongs to

Reasons for failure propagate upwards, meaning that if a child node fails the parent will also contain the same reason.

Now let's take a look at the arguments of the plan we just created.
```python
print(plan.root.status)
print(plan.root.start_time)
print(plan.root.plan)
```

## Plan Execution
Plans can be executed using the `perform` method. This method will execute the plan and also perform all the resolution
of Action Designators.

```python

with simulated_robot:
    plan.perform()
```

This will execute the plan in a simulated environment. Now we can take a look at the arguments of the plan after execution.

```python
print(plan.root.status)
print(plan.root.children[0].status)
print(plan.root.start_time)
print(plan.root.end_time)
print(plan.root.reason)
```