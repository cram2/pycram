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

# Ontologies in PyCRAM

Cognitive Architectures often include some logical theory to them that enables higher level reasoning, and so does 
PyCRAM. The PyCRAM typology PyCRAP is an ontology that can be used in PyCRAM to describe the belief of the robot
using concepts. These individuals for the ontology are created on the fly, and PyCRAP can then be used to for instance 
filter for certain objects.

PyCRAP is defined in its package next to the PyCRAM package and hence allows users and developers to
add new classes, instances, and relationships on demand. 
Furthermore, this architecture allows the users to directly see what's in the ontology and how it is structured without
switching to tools like [Protégé](https://protege.stanford.edu/).
The linter and AI assistants like Copilot can also deal with this notation and guide the users without annoyances.

PyCRAP contains a subpackage with ontologies. 
Every package in the ontologies subpackage is a separate ontology.
The classes, properties, and so on can just be included via the import statement.

PyCRAP also comes with a parser to reflect any ontology that can be loaded with owlready2 in python files.
This is very similar to the [code generation tool of sqlalchemy](https://github.com/agronholm/sqlacodegen).

Note that this area is under construction and may frequently change.

## Usage

You can access the ontology by importing the PyCRAP package:

```python
import pycram
```

The ontology is structured in classes and instances. The classes are the concepts that are used to describe the world.
The individuals in the ontology can be created when spawning objects in the world. 
For example, when you spawn a milk or a cereal box.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import WorldMode
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import PoseStamped
from pycrap.ontologies import Milk, Cereal, Food

world = BulletWorld(mode=WorldMode.DIRECT)

milk = Object("milk", Milk, "milk.stl")
cereal = Object("cereal", Cereal, "breakfast_cereal.stl", pose=PoseStamped.from_list([1.4, 1, 0.95]))
```

You can query the ontology using [owlready2](https://owlready2.readthedocs.io/en/v0.41/index.html). 
For example, we can see all food objects like this:

```python
print("All food instances", list(world.ontology.search(type=Food)))
```

You can also search for objects in the world using the ontology:

```python
from pycram.designators.object_designator import OntologyObjectDesignatorDescription
object_designator = OntologyObjectDesignatorDescription(world.ontology.search(type=Food))
result_in_world = list(object_designator.__iter__())
print(result_in_world)
```

Architecturally speaking, the ontology is a part of the world and is accessible through the world object.
Objects created in a world will only appear in the ontology of that world.

## Extending the Ontology

Feel free to extend the ontology with new classes and relations as you go! 
These will be then reviewed in the pull requests so don't be shy!

If you are looking to integrate PyCRAP with some other ontology like [Soma](https://ease-crc.github.io/soma/), you
can do so by denoting the class equivalences in the PyCRAP ontology. 
Details on this are found in the [owlready2 properties documentation](https://owlready2.readthedocs.io/en/v0.41/properties.html#obtaining-indirect-relations-considering-subproperty-transitivity-etc).

Currently, only objects spawned during runtime are tracked in the A-Box of PyCRAP.

The roadmap for a full integration is as follows:
    - Map the entire belief state in the A-Box, including
        - Relevant links from the URDF
        - Robot descriptions
    - Find a way to get action descriptions from ontological statements
    - Use the ontology to guide the robot in its decision making
    
