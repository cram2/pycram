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

# Plan Language

The PyCRAM plan language is a way to structure the execution of your plan. In generally the plan language allows to
execute designators either sequential or in parallel. Furthermore, exceptions that occur during execution of a plan with
the plan language do not interrupt the execution instead they are caught and saved to a dictionary for later analysis.
All language expressions return a State, this can either be SUCCEDED or FAILED.

There are 4 language expressions:

| Expression | Name             | Description                                                                                                                                                                                                                                                                                | 
|------------|------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| +          | **Sequential**   | Executes the designators one after another, if one of the designators raises an exception the execution is aborted and the state FAILED will be returned.                                                                                                                                  |
| -          | **Try In Order** | Executes the designators one after another, if one designator raises an exception the exception is caught and saved but the execution is not interrupted and the other designators are executed. Returns the state SUCCEDED if at least one designator can be executed without exception. |
| *          | **Repeat**       | Repeat the previous language expression a number of time. Has to be used with a language expression and an integer.                                                                                                                                                                        | 
| \|         | **Parallel**     | Executes all designators in parallel. For each designator there will be a new thread created and the designator is executed in this thread. If one of the designators raises an exception the returned state will be FAILED.                                                               |
| ^          | **Try All**      | Executes all designators in parallel with a designated thread for each designator. Returns the state SUCCEDED if at least one designator can be executed without an exception                                                                                                              |
| >>         | **Monitor**      | Monitors the execution of the attached langauge expression, will interrupt the execution as soon as a given condition is fulfilled.                                                                                                                                                        | 

The Sequential expression is the only one which aborts the execution once an error is raised.

When using the plan language a tree structure of the plan is created where the language expressions are nodes and
designators are leafs. This tree uses AnyTree (like the task tree) and can be rendered with the anytree Renderer.

## Sequential

This language expression allows to execute designators one after another, if one of the designators raises an exception
the execution will be aborted and the state FAILED will be returned.

We will start with a simple example that uses an action designator for moving the robot and parking its arms.

```python
import time

from pycram.designators.action_designator import *
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import Arms

navigate = NavigateActionDescription([Pose([1, 1, 0])])
park = ParkArmsActionDescription([Arms.BOTH])

plan = navigate + park
```

With this simple plan created we can inspect it and render the created tree structure.

```python
from anytree import RenderTree

print(RenderTree(plan))
```

As you can see there is the root node which is the language expression and then there are the leafs which are the
designators. When executing this plan the Sequential node will try to execute the NavigateAction and if that is finished
without any error the ParkArmsAction will be executed.

The plan can be executed by wrapping it inside a ```with simulated_robot``` environment and calling perform on the
plan.

If you are performing a plan with a simulated robot, you need a BulletWorld.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import ObjectType
from pycrap.ontologies import Robot

world = BulletWorld()
pr2 = Object("pr2", Robot, "pr2.urdf")
```

```python
from pycram.process_module import simulated_robot

world.reset_world()

with simulated_robot:
    plan.perform()
```

## Try In Order

Try in order is similar to Sequential, it also executes all designators one after another but the key difference is that
an exception in one of the designators does not terminate the whole execution. Furthermore, the state FAILED will only
be returned if all designator executions raise an error.

Besides the described difference in behaviour this language expression can be used in the same way as Sequential.

```python
from pycram.designators.action_designator import *
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import Arms
from pycram.process_module import simulated_robot

world.reset_world()

navigate = NavigateActionDescription([Pose([1, 1, 0])])
park = ParkArmsActionDescription([Arms.BOTH])

plan = navigate - park

with simulated_robot:
    plan.perform()
```

## Parallel

Parallel executes all designator at once in dedicated threads. The execution of other designators is not aborted when a
exception is raised, this is the case since threads can not be killed from the outside and this would also cause
unforeseen problems. The state returned will be SUCCEDED if all designators could be executed without an exception raised
in any other case FAILED will be returned.

Since executing designators in parallel can get chaotic especially with complex actions like PickUp or Transport. For
this reason not all action designators can be used in parallel and try all expressions. The list of action designator
that cannot be used in language expressions can be seen in {attr}`~pycram.language.Language.parallel_blocklist`.

Designators that cannot be used in parallel and try all:

* PickUpAction
* PlaceAction
* OpenAction
* CloseAction
* TransportAction

Using the parallel expressions works like Sequential and TryInOrder.

```python
from pycram.designators.action_designator import *
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import Arms
from pycram.process_module import simulated_robot

world.reset_world()

navigate = NavigateActionDescription([Pose([1, 1, 0])])
park = ParkArmsActionDescription([Arms.BOTH])

plan = navigate | park

with simulated_robot:
    plan.perform()
```

## Try All

TryAll is to Parallel what TryInOrder is to Sequential, meaning TryAll will also execute all designators in parallel but
will return SUCCEEDED if at least one designator is executed without raising an exception.

TryAll can be used like any other language expression.

```python
from pycram.designators.action_designator import *
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import Arms
from pycram.process_module import simulated_robot

world.reset_world()

navigate = NavigateActionDescription([Pose([1, 1, 0])])
park = ParkArmsActionDescription([Arms.BOTH])

plan = navigate ^ park

with simulated_robot:
    plan.perform()
```

## Combination of Expressions

You can also combine different language expressions to further structure your plans. If you combine sequential and
parallel expression please keep in mind that sequential expressions bind stronger than parallel ones. For example:

```
navigate | park + move_torso
```

In this case 'park' and 'move_torso' would form a Sequential expression and 'naviagte' would form a Parallel expression
with Sequential. You can try this yourself in the following cell.

```python
from pycram.designators.action_designator import *
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import Arms
from pycram.process_module import simulated_robot

world.reset_world()

navigate = NavigateActionDescription([Pose([1, 1, 0])])
park = ParkArmsActionDescription([Arms.BOTH])
move_torso = MoveTorsoActionDescription([TorsoState.HIGH]) 

plan = navigate | park + move_torso

with simulated_robot:
    plan.perform()
```

## Code Objects

You can not only use designators in the plan language but also python code. For this there is the {class}`~pycram.language.Code`  object
which takes a callable and the arguments for this callable. This allows you to execute arbitrary code in a plan.

The callable that is used in the {class}`~pycram.language.Code` object can either be a lambda expression or, for more complex code, a
function. If you use a function you can provide parameters as keyword-arguments.

```python
from pycram.designators.action_designator import *
from pycram.datastructures.enums import Arms
from pycram.process_module import simulated_robot
from pycram.language import Code


def code_test(param):
    print("-" * 20)
    print(param)


park = ParkArmsActionDescription([Arms.BOTH])
code = Code(lambda: print("This is from the code object"))
code_func = Code(code_test, {"param": "Code function"})

plan = park | code | code_func

with simulated_robot:
    plan.perform()
```

## Exception Handling

If an exception is raised during the execution of a designator when it is used in a language expression the exception
will be caught and saved to a dictionary. In general all designators in a language expression are executed regardless
of exceptions raised, the only exception from this is the Sequential expression which stops after it encountered an
exception.

The language will only catch exceptions that are of type {class}`~pycram.plan_failures.PlanFailure` meaning errors that are defined in
plan_failures.py in PyCRAM. This also means normal Python errors, such as KeyError, will interrupt the execution of your
designators.

We will see how exceptions are handled at a simple example.

```python
from pycram.designators.action_designator import *
from pycram.process_module import simulated_robot
from pycram.language import Code
from pycram.failures import PlanFailure


def code_test():
    raise PlanFailure


navigate = NavigateActionDescription([Pose([1, 1, 0])])
code_func = Code(code_test)

plan = navigate | code_func

with simulated_robot:
    plan.perform()

print(plan.exceptions)
```

## Repeat

Repeat simply repeats a language expression a number of times. As all other language expressions Repeat captures
exceptions that occur during execution and saves them to the dictionary in the root of the plan.

Since Repeat uses the \* operator you should keep in mind that it will be evaluated before any other operator, so use
parentheses to ensure the correct structure of your plan.

You can see an example of how to use Repeat below.

```python
from pycram.designators.action_designator import *
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import TorsoState

move_torso_up = MoveTorsoActionDescription([TorsoState.HIGH])
move_torso_down = MoveTorsoActionDescription([TorsoState.LOW])

plan = (move_torso_up + move_torso_down) * 5

with simulated_robot:
    plan.perform()
```

## Monitor

Monitor allows to monitor the execution of a language expression and interrupt it as soon as a given condition is
fulfilled. The condition can either be a Callable which returns a boolean or a Fluent.
When executed the Monitor will create a separate thread which will check if the condition is satisfied with a frequency
of 10 Hz. If the condition is satisfied the execution of the language expression will be interrupted.

For the example on how Monitors work we will use the previous example with the robot moving up and down. We will use a
Monitor to interrupt the execution after 2 seconds instead of executing the whole plan 5 times.

```python
from pycram.designators.action_designator import *
from pycram.process_module import simulated_robot
from pycram.language import Monitor
import time
from pycram.datastructures.enums import TorsoState

move_torso_up = MoveTorsoActionDescription([TorsoState.HIGH])
move_torso_down = MoveTorsoActionDescription([TorsoState.LOW])


def monitor_func():
    time.sleep(2)
    return True


plan = (move_torso_up + move_torso_down) * 5 >> Monitor(monitor_func)

with simulated_robot:
    plan.perform()
```

If you are finished with this example you can close the world with the cell below.

```python
world.exit()
```
