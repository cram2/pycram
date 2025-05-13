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

# Object Designator

Object designators are used to describe objects located in the BulletWorld or the real environment and then resolve them
during runtime to concrete objects.

Object designators are different from the Object class in bullet_world.py in the way that they just describe an object
and do not create objects or provide methods to manipulate them. Nevertheless, object designators contain a reference to
the BulletWorld object.

An Object designator takes two parameters, of which at least one has to be provided. These parameters are:

* A list of names
* A list of types

Object Designators work similar to Location designators, they get constrains describing a set of objects and when
resolved return a specific instance.

For all following examples we need a BulletWorld, so let's create one.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import PoseStamped

world = BulletWorld(WorldMode.DIRECT)
```

## Believe Object

This object designator is used to describe objects that are located in the BulletWorld. So objects that are in the
belief state, hence the name. In the future when there is a perception interface, there will be a ```RealObject```
description which will be used to describe objects in the real world.

Since {meth}`~pycram.designators.object_designator.BelieveObject` describes Objects in the BulletWorld we create a few.

```python
from pycrap.ontologies import Milk, Cereal, Kitchen, Spoon
kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([1.3, 1, 0.9]))
cereal = Object("froot_loops", Cereal, "breakfast_cereal.stl", pose=PoseStamped.from_list([1.3, 0.9, 0.95]))
spoon = Object("spoon", Spoon, "spoon.stl", pose=PoseStamped.from_list([1.3, 1.1, 0.87]))
```

Now that we have objects we can create an object designator to describe them. For the start we want an object designator
only describing the milk. Since all objects have unique names we can create an object designator using a list with only
the name of the object.

```python
from pycram.designators.object_designator import BelieveObject

object_description = BelieveObject(names=["milk"])

print(object_description.resolve())
```

You can also use the type to describe objects, so now we want to have an object designator that describes every food in
the world.

```python
from pycram.designators.object_designator import BelieveObject

object_description = BelieveObject(types=[Milk, Cereal])

print(object_description.resolve())
```

## Object Part

Part of object designators can be used to describe something as part of another object. For example, you could describe
a specific drawer as part of the kitchen. This is necessary since the drawer is no single BulletWorld Object but rather
a link of the kitchen which is a BulletWorld Object.

For this example we need just need the kitchen, if you didn't spawn it in the previous example you can spawn it with the
following cell.

```python
from pycram.designators.object_designator import ObjectPart, BelieveObject

kitchen_desig = BelieveObject(names=["kitchen"]).resolve()

object_description = ObjectPart(names=["sink_area_left_upper_drawer_main"], part_of=kitchen_desig)

print(object_description.resolve())
```

## Object Designators as Generators

Similar to location designators object designators can be used as generators to iterate through every object that they
are describing. We will see this at the example of an object designator describing every type of food.

For this we need some objects, so if you didn't already spawn them you can use the next cell for this.

```python
from pycram.designators.object_designator import BelieveObject

object_description = BelieveObject(types=[Milk, Cereal])

for obj in object_description:
    print(obj, "\n")
```

To close the world use the following exit function.

```python
world.exit()
```
