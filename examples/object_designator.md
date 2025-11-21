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
from pycram.testing import setup_world

world = setup_world()
```

# Using the Entity Query Language
To query the belief state for certain objects we use the Entity Query Language (EQL) which is part of the KRROOD project. 
For a more detailed documentation check out their website (https://github.com/code-iai/krrood/tree/main).

To query for a body for example the milk bottle we need to create a query. 

```python
from krrood.entity_query_language.entity import an, entity, contains, let
from krrood.entity_query_language.symbolic import symbolic_mode
from semantic_digital_twin.world_description.world_entity import Body

with symbolic_mode():
    query = an(entity(body := let(type_=Body, domain=world.bodies),
                                  contains(body.name.name, "milk")))
```
This query searches in all bodies of the world, this is defined by the ```let``` in the first line. The next lines define 
constrains of this body, in this case we check the name of each body if it contains the string "milk". 

This only defines a query but does not evaluate it. To evaluate the query and get a body satisfying the constrains we 
can just call ```evaluate``` on it. 

```python
print(query.evaluate())
```


## Object Designators as Generators

Depending on the query there could be more than one solution. For example a query searching for all bodies whose name 
contains the sub-string "cabinet" would yield multiple results. 

We first need a query with multiple results. 

```python
from krrood.entity_query_language.entity import an, entity, contains, let
from krrood.entity_query_language.symbolic import symbolic_mode
from semantic_digital_twin.world_description.world_entity import Body

with symbolic_mode():
    query = an(entity(body := let(type_=Body, domain=world.bodies),
                                  contains(body.name.name, "cabinet")))
```

```python
for cabinet in query.evaluate():
    print(cabinet)
```
