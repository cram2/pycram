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

# Pose

Poses in PyCRAM are represented by the Pose class which inherits from the PoseStamped message of ROS. This makes PyCRAMs
poses compatible with everything in ROS like services, topics or TF.

This notebook will provide an overview about poses, how to use them and what they can do. We will start by simply
creating a pose.

Before we start a few words about naming convention of Poses in PyCRAM. Naming convention is similar to the PoseStamped
message so if you are familiar with that this should be easy.

* **Position:** A position means the position in cartesian space, so the x, y, and z coordinates.
* **Orientation:** An orientation is the rotation in all three axes represented as a quaternion with x, y, z, w.
* **Pose:** A pose is the combination of a position and an orientation. Poses in PyCRAM also contain a frame of
  reference to which the position and orientation are relative.

```python
from pycram.datastructures.pose import PoseStamped

example_pose = PoseStamped.from_list([1, 2, 3], [0, 0, 0, 1], "map")
print(example_pose)
```

As you can see we created the ```example_pose``` with a position of ```[1, 2, 3]``` and an orientation
of ```[0, 0, 0, 1]``` in the frame ```map```. But we don't need to provide all these parameters for a Pose, in case
there is no parameter the Pose will use default parameter.

```python
from pycram.datastructures.pose import PoseStamped

default_pose = PoseStamped()
print(default_pose)
```

In case no parameter is provided the defualt parameter are:

* position: ```[0, 0, 0]```
* orientation: ```[o, 0, 0, 1]```
* frame: ```map```

The following example will show how to access the data stored in a pose.

```python
from pycram.datastructures.pose import PoseStamped

example_pose = PoseStamped.from_list([1, 2, 3], [0, 0, 0, 1], "map")

print(f"Access to a component of the position: {example_pose.position.y}")

print(f"Access to a component of the rotation: {example_pose.orientation.x}")

print(f"Get the whole position as geometry_msgs/Pose:\n{example_pose.position}")

print(f"You can also get position or orientation as a list: {example_pose.position.to_list()}")

print(f"Same with the whole pose: {example_pose.to_list()}")

print(f"Access the reference frame: {example_pose.frame_id}")
```

## Editing a pose

You can also edit the data saved in a Pose, similar to how you access it.

```python
from pycram.datastructures.pose import PoseStamped

example_pose = PoseStamped.from_list([1, 2, 3], [0, 0, 0, 1], "map")

# Edit a single component of the position 
example_pose.position.x = 3
print(f"Edit only one component:\n{example_pose.position}", "\n")

# Edit the whole position
example_pose.position = [0, 0, 1]
print(f"Edit the whole position:\n{example_pose.position}", "\n")

example_pose.frame_id = "new_frame"
print(f"Set a new frame:\n{example_pose.frame_id}", "\n")

example_pose.set_position([3, 2, 1])
print(f"Set the position via method:\n{example_pose.position}", "\n")
```

## Copy Poses

You can also copy Poses to create a new Pose with the same data. This can be useful if you have a method which would
need to alter the Pose, since poses are passed by reference to a method every change done to the Pose in the method
would affect the instanced passed to the method.

```python
from pycram.datastructures.pose import PoseStamped

example_pose = PoseStamped.from_list([1, 2, 3], [0, 0, 0, 1], "map")

copy_pose = example_pose.copy()

print(example_pose, "\n")
print(copy_pose)
```

## Convert to Transform

PyCRAM also has its own transform at which we will take a look in the next section. However, here we will take a look at
how to convert a Pose into a Transform.

For this example we will take a Pose which represents the current pose of a milk object and convert it into a Transform
which represents the transformation from the ```map``` frame to the ```milk``` frame.

```python
from pycram.datastructures.pose import PoseStamped

milk_pose = PoseStamped.from_list([3, 4, 1], [1, 0, 0, 1], "map")

milk_transform = milk_pose.to_transform_stamped("milk")

print(milk_transform)
```

# Transforms

Transforms are similar to Poses but instead of representing a Pose in a frame of reference they represent a
transformation from one frame of reference to another. For this purpose Transforms have an additional parameter
called ```child_frame_id``` which is the frame of reference to which the Transform is pointing.

Transforms in PyCRAM inherit from the TransformStamped message of ROS which makes them, like Poses, compatible to ROS
services and topics that expect a TransformStamped message. Therefore, the naming conventions of Transforms are the same
as of TransformStamped which.

* **Translation:** The vector describing the transformation in cartesian space.
* **Rotation:** The quaternion describing the transformation of rotation.
* **Transform:** The combination of translation and rotation

```python
from pycram.datastructures.pose import TransformStamped

example_transform = TransformStamped.from_list([1, 2, 2], [0, 0, 0, 1], "map", "object")

print(example_transform)
```

Transforms have the same methods to get and set values as Poses have, therefore only a short showcase will be given. For
more details please look at the Pose example or the API documentation.

```python
from pycram.datastructures.pose import TransformStamped

example_transform = TransformStamped.from_list([2, 5, 1], [0, 0, 1, 1], "map", "object")

print(f"Access the rotation:\n{example_transform.rotation}", "\n")

print(f"Access the child_frane: {example_transform.child_frame_id}", "\n")

# changing translation and rotation is exactly like with Poses.

example_transform.translation = [1, 1, 1]
print(f"New translation:\n{example_transform.translation}")
```

## Convert to Pose and Copy

Analog to Poses Transforms have a method that converts a Transform to a Pose, in this process the ```child_frame_id```
will be lost.

Also like in Poses Transforms have a ```copy``` method which creates an exact copy of this Transform.

```python
from pycram.datastructures.pose import TransformStamped

milk_transform = TransformStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map", "milk")

milk_pose = milk_transform.to_pose_stamped()

print(f"The converted pose:\n{milk_pose}", "\n")

example_transform = TransformStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map", "milk")

copy_transform = example_transform.copy()

print(f"The copied transform:\n{copy_transform}")
```

## Operations on Transforms

Transforms have, unlike Poses, operations that can be done. These operations are:

* Multiplication
* Invert
* InverseTimes

### Multiplication

We will first take a look at the multiplication of Transforms. We will use an example were we have two Transforms, the
first from ```map``` to a ```hand``` frame and the second from the ```hand``` to a ```milk``` frame. By multiplying
these two we get the Transform from ```map``` to ```milk``` frame.

```python
from pycram.datastructures.pose import TransformStamped

map_to_hand = TransformStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map", "hand")

hand_to_milk = TransformStamped.from_list([0.1, 0.05, 0], [0, 0, 0, 1], "hand", "milk")

map_to_milk = map_to_hand * hand_to_milk

print(map_to_milk)
```

### Invert

This inverts a Transform, so in we have a transform from ```map``` to ```milk``` then inverting it results in a
Transform from ```milk``` to ```map``` .

```python
from pycram.datastructures.pose import TransformStamped

map_to_milk = TransformStamped.from_list([1, 1, 0.5], [0, 0, 0, 1], "map", "milk")

milk_to_map = map_to_milk.invert()

print(milk_to_map)
```

### Inverse Times

Inverse times combines the inverting and multiplication of Transforms, this results in a 'minus' for Transforms. We will
again use the example of a hand holding a milk, but this time we have the Transforms from ```map``` to ```milk```
and ```hand``` to ```milk```.

```python
from pycram.datastructures.pose import TransformStamped

map_to_milk = TransformStamped.from_list([1.1, 1.05, 1], [0, 0, 0, 1], "map", "milk")

hand_to_milk = TransformStamped.from_list([0.1, 0.05, 0], [0, 0, 0, 1], "hand", "milk")

map_to_milk = map_to_milk.inverse_times(hand_to_milk)

print(map_to_milk)
```
