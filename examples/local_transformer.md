---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.3
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

# Local Transformer

The local transformer is used to handle transforms between different frames in PyCRAM. This is useful when you want to
transform a pose from one frame to another, for example, from the map frame to the frame of an object. This example will
introduce the Local Transformer and how to use it to transform poses between frames.

## Setting up the Environment

This step involves importing the required modules and initializing key components for our tasks.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import TransformStamped, PoseStamped
from pycram.local_transformer import LocalTransformer
from pycram.datastructures.enums import WorldMode
```

## Initializing the World

Every robot simulation requires a world where it can interact. This world serves as the playground where the robot
performs tasks.
Let's start by creating this world.

Since the local transformer can only transform between frames of objects which are in the world, we need to create a
world first.

```python
# Create an instance of the BulletWorld
world = BulletWorld(WorldMode.DIRECT)
```

## Adding Objects to the World

For our robot to perform meaningful tasks, we need to populate its world with objects.
In this section, we'll add a variety of objects, from a simple floor plane to kitchen setups and items like milk and
bowls.
These objects will be used in subsequent tasks, to provide the frames to which we will transform poses.

```python
from pycram.worlds.bullet_world import Object
from pycram.datastructures.enums import ObjectType
from pycrap.ontologies import Kitchen, Milk, Bowl

kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([0.9, 1, 0.95]))
bowl = Object("bowl", Bowl, "bowl.stl", pose=PoseStamped.from_list([1.6, 1, 0.90]))
```

## Creating a Local Transfomer

The local transformer is implemented as a singelton, meaing regardless of how much and where an instance is created it
will always be the same instance. This is done since the local transfomer collects all transformations between frames
and would there always be a new instance, all transformations woulb need to be re-collected.

```python
from pycram.local_transformer import LocalTransformer

local_transformer = LocalTransformer()
print(local_transformer)

new_local_transformer = LocalTransformer()
print(new_local_transformer)
```

## Transformations with LocalTransformer

Now that we have our world set up, let's perform some transformations. We'll use the LocalTransformer to transform poses
relative to our objects.

```python
from pycram.local_transformer import LocalTransformer

l = LocalTransformer()
test_pose = PoseStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map")

transformed_pose = l.transform_to_object_frame(test_pose, milk)
print(transformed_pose)

print("-------------------")
new_pose = l.transform_pose(transformed_pose, "map")
print(new_pose)
```

In the above code, we first transformed a pose to the object frame of the milk object, and then we transformed it back
to the map frame. This demonstrates how we can easily manipulate poses relative to objects in our environment.
You can also transform poses relative to other poses. by using the transform_pose method. Further you can set a
Transform.

```python
from pycram.datastructures.pose import TransformStamped

l.update_transforms([TransformStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map", "test_frame")])
p = PoseStamped()

transformed_pose = l.transform_pose(p, "test_frame")
```

## Transformation frames

Links of an Object are represented by the Object frame_id + '/' + link name. Since link names need to be
unique for an URDF this is no problem.

These frames need to be used in whenever you are transforming something with the local transformer. To get the base
frame of an Object, meaning the frame name without any link, there is the attribute tf_frame and for the frame of a link
there is a method which returns the frame name of a link given the link name.

```python
print(milk.tf_frame)

print(kitchen.get_link_tf_frame("kitchen_island_surface"))
```

You can use the cell below to exit the simulation.

```python
world.exit()
```
