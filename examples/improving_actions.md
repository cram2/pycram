---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.2
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

# Improving Actions using Probabilities

In this tutorial we will look at probabilistic specifications of actions and especially at an advance plan to pick up
objects.
After this tutorial you will know:

- Why are probabilities useful for robotics
- How to use probabilistic models to specify actions
- How to use probabilistic machine learning to improve actions

Let's start by importing all the necessary modules.

```python
import numpy as np
import os
import random

import pandas as pd
import sqlalchemy.orm

import plotly

plotly.offline.init_notebook_mode()
import plotly.graph_objects as go
import tqdm

from probabilistic_model.learning.jpt.jpt import JPT
from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe
from random_events.product_algebra import Event, SimpleEvent

import pycram.orm.base
from pycram.designators.action_designator import MoveTorsoActionPerformable
from pycram.failures import PlanFailure
from pycram.designators.object_designator import ObjectDesignatorDescription
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.robot_descriptions import robot_description
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.process_module import ProcessModule, simulated_robot
from pycram.designators.specialized_designators.probabilistic.probabilistic_action import MoveAndPickUp, Arms, Grasp
from pycram.tasktree import task_tree, TaskTree 

ProcessModule.execution_delay = False
np.random.seed(69)
random.seed(69)
```

Next, we connect to a database where we can store and load robot experiences.

```python
pycrorm_uri = "robot_enjoyer:I_love_robots_123@neem-2.informatik.uni-bremen.de:3306/pycram_ci"
pycrorm_uri = "mysql+pymysql://" + pycrorm_uri
engine = sqlalchemy.create_engine(pycrorm_uri)
session = sqlalchemy.orm.sessionmaker(bind=engine)()
# pycram.orm.base.Base.metadata.create_all(engine)
```

Now we construct an empty world with just a floating milk, where we can learn about PickUp actions.

```python
world = BulletWorld(WorldMode.DIRECT)
print(world.prospection_world)
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
viz_marker_publisher = VizMarkerPublisher()
milk_description = ObjectDesignatorDescription(types=[ObjectType.MILK]).ground()
```

Next, we create a default, probabilistic model that describes how to pick up objects. We visualize the default policy.
The default policy tries to pick up the object by standing close to it, but not too close.

```python
fpa = MoveAndPickUp(milk_description, arms=[Arms.LEFT, Arms.RIGHT],
                    grasps=[Grasp.FRONT.value, Grasp.LEFT.value, Grasp.RIGHT.value, Grasp.TOP.value])
print(world.current_world)
p_xy = fpa.policy.marginal([fpa.variables.relative_x, fpa.variables.relative_y])
fig = go.Figure(p_xy.root.plot(), p_xy.root.plotly_layout())
fig.update_layout(title="Marginal View of relative x and y position of the robot with respect to the object.")
fig.show()
```

Next, we will perform pick up tasks using the default policy and observe the success rate.
The robot will now experiment with the behaviour specified by the default policy and observe his success rate in doing
so.
After finishing the experiments, we insert the results into the database.

```python
# pycram.orm.base.ProcessMetaData().description = "Experimenting with Pick Up Actions"
# fpa.sample_amount = 100
# with simulated_robot:
#     fpa.batch_rollout()
# task_tree.root.insert(session)
# session.commit()
# task_tree.reset_tree()
```

Let's query the data that is needed to learn a pick up action and have a look at it.

```python
samples = pd.read_sql(fpa.query_for_database(), engine)
samples["arm"] = samples["arm"].astype(str)
samples["grasp"] = samples["grasp"].astype(str)
samples
```

We can now learn a probabilistic model from the data. We will use the JPT algorithm to learn a model from the data.

```python
variables = infer_variables_from_dataframe(samples, scale_continuous_types=False, 
                                           min_samples_per_quantile=2, 
                                           min_likelihood_improvement = 0.)
model = JPT(variables, min_samples_leaf=25)
model.fit(samples)
model = model.probabilistic_circuit
print(model)
```

```python
arm, grasp, relative_x, relative_y = model.variables
```

Let's have a look at how the model looks like. We will visualize the model density when we condition on grasping the
object from the front with the left arm.

```python
event = SimpleEvent({arm: Arms.LEFT, grasp: Grasp.FRONT}).as_composite_set()
conditional_model, conditional_probability = model.conditional(event)
p_xy = conditional_model.marginal([relative_x, relative_y])
fig = go.Figure(p_xy.plot(), p_xy.plotly_layout())
fig.show()
```

Let's make a monte carlo estimate on the success probability of the new model.

```python
fpa.policy = model
fpa.sample_amount = 5
with simulated_robot:
    fpa.batch_rollout()
```

We can see, that our new and improved model has a better success probability as opposed to the 30% from the standard
policy.

Next, we put the learned model to the test in a complex environment, where the milk is placed in a difficult to access
area.

```python
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "apartment.urdf")

milk.set_pose(Pose([0.5, 3.15, 1.04]))
milk_description = ObjectDesignatorDescription(types=[ObjectType.MILK]).ground()
fpa = MoveAndPickUp(milk_description, arms=[Arms.LEFT, Arms.RIGHT],
                    grasps=[Grasp.FRONT, Grasp.LEFT, Grasp.RIGHT, Grasp.TOP], policy=model)
fpa.sample_amount = 200

```

```python
p_xy = model.marginal([relative_x, relative_y])
fig = go.Figure(p_xy.plot(), p_xy.plotly_layout())
fig.show()
```

Let's look at the density of the relative x and y position of the robot with respect to the milk. We can see that he
would like to access the object from the right front area.

```python
grounded_model = fpa.ground_model()
p_xy = grounded_model.marginal([relative_x, relative_y]).simplify()
fig = go.Figure(p_xy.plot(), p_xy.plotly_layout())
fig.update_layout(title="Marginal View of relative x and y position with respect to the milk",
                  xaxis_range=[-1, 1], yaxis_range=[-1, 1])
fig.show()
```

Finally, we observe our improved plan in action.

```python
from pycram.designators.action_designator import ParkArmsActionPerformable

world.reset_world()
milk.set_pose(Pose([0.5, 3.15, 1.04]))
with simulated_robot:
    MoveTorsoActionPerformable(0.3).perform()
    for sample in fpa:
        try:
            ParkArmsActionPerformable(Arms.RIGHT).perform()
            sample.perform()
            break
        except PlanFailure as e:
            continue
```

```python
world.exit()
viz_marker_publisher._stop_publishing()
```
