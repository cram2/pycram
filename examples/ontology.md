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

Cognitive Architectures often include some logical theory to them that enables a higher level reasoning and so does 
PyCRAM. The PyCRAM typology PyCRAP is an ontology that can be used in PyCRAM to describe the belief of the robot
using concepts. These individuals for the ontology are created on the fly and PyCRAP can then be used to for instance 
filter for certain objects.

PyCRAP is defined in it's package next to the PyCRAM package and hence allows users and developers to
add new classes, instances, and relationships on demand. 
Furthermore, this architecture allows the users to directly see whats in the ontology and how it is structured.
The linter and AI assistants like Copilot can also deal with this notation and guide the users without annoyances.

Note that this area is under construction and may frequently changes.

You can access the ontology by importing the PyCRAP package:

```python
import pycram
import pycrap
```

The ontology is structured in classes and instances. The classes are the concepts that are used to describe the world.
The individuals in the ontology can be created when spawning objects in the world. 
For example, when you spawn a milk or a cereal box.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import WorldMode
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import Pose

world = BulletWorld(mode=WorldMode.DIRECT)

milk = Object("milk", pycrap.Milk, "milk.stl")
cereal = Object("cereal", pycrap.Cereal, "breakfast_cereal.stl", pose=Pose([1.4, 1, 0.95]))
```

You can query the ontology using owlready2. For example, we can see all food objects like this:

```python
print("All food instances", list(filter(lambda x: x in pycrap.Food.instances(), pycrap.ontology.individuals())))
```

Feel free to extend the ontology with new classes and relations as you go! 
These will be then reviewed in the pull requests so don't be shy!