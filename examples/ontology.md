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

<!-- #region jupyter={"outputs_hidden": false} -->

# Ontology interface

This tutorial demonstrates basic usages of __owlready2__ API for ontology manipulation. Notably, new ontology concept
triple classes (subject, predicate, object) will be dynamically created, with optional existing ontology parent classes
that are loaded from an OWL ontology. Then through the interconnected relations specified in triples, designators and
their corresponding ontology concepts can be double-way queried for input information in certain tasks, eg. making a
robot motion plan.
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
from pathlib import Path
from typing import Type
from pycram.designator import ObjectDesignatorDescription
```

<!-- #region jupyter={"outputs_hidden": false} -->

# Owlready2

[Owlready2](https://owlready2.readthedocs.io/en/latest/intro.html) is a Python package providing a transparent access to
OWL ontologies. It supports various manipulation operations, including but not limited to loading, modification, saving
ontologies. Built-in supported reasoners include [HermiT](http://www.hermit-reasoner.com)
and [Pellet](https://github.com/stardog-union/pellet).
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
import logging

try:
    from owlready2 import *
except ImportError:
    owlready2 = None
    logging.error("Could not import owlready2, Ontology Manager could not be initialized!")

logging.getLogger().setLevel(logging.INFO)
```

<!-- #region jupyter={"outputs_hidden": false} -->

# Ontology Manager

{class}`~pycram.ontology.ontology.OntologyManager` is the singleton class acting as the main interface between PyCram with ontologies, whereby object
instances in the former could query relevant information based on the semantic connection with their corresponding
ontology concepts.

Such connection, as represented by triples (subject-predicate-object), could be also created on the fly if not
pre-existing in the loaded ontology.

Also new and updated concepts with their properties defined in runtime could be stored into
an [SQLite3 file database](https://owlready2.readthedocs.io/en/latest/world.html) for reuse.

Here we will use [SOMA ontology](https://ease-crc.github.io/soma) as the baseline to utilize the generalized concepts
provided by it.
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
from pycram.ontology.ontology import OntologyManager, SOMA_HOME_ONTOLOGY_IRI
from pycram.ontology.ontology_common import OntologyConceptHolderStore, OntologyConceptHolder

ontology_manager = OntologyManager(SOMA_HOME_ONTOLOGY_IRI)
main_ontology = ontology_manager.main_ontology
soma = ontology_manager.soma
dul = ontology_manager.dul
```

[General class axioms](https://owlready2.readthedocs.io/en/latest/general_class_axioms.html) of the loaded ontologies
can be queried by

```python
print(f"{main_ontology.name}: ", ontology_manager.get_ontology_general_class_axioms(main_ontology))
print(f"{soma.name}: ", ontology_manager.get_ontology_general_class_axioms(soma))
print(f"{dul.name}: ", ontology_manager.get_ontology_general_class_axioms(dul))
```

<!-- #region jupyter={"outputs_hidden": false} -->

## Ontology Concept Holder

__OntologyConceptHolder__ class, encapsulating an __owlready2.Thing__ instance, is used primarily as the binding
connection between the `owlready2.Thing` ontology concept to PyCram designators. We make it that way, instead of
creating a custom concept class that inherits from `owlready2.Thing` for the reasons below:

- `owlready2` API does not have very robust support for client classes to inherit from theirs with added (non-semantic)
  attributes, particularly in our case, where classes like {class}`~pycram.designator.DesignatorDescription` have their `metaclass` as `ABCMeta`,
  while it is `EntityClass` that is the metaclass used for basically all concepts (classes, properties) in `owlready2`.
  Since those two metaclasses just bear no relationship, for the inheritance to work, the only way is to create a child
  metaclass with both of those as parents, however without full support by `owlready2`, plus the second reason below
  will point out it's not worth the effort.


- Essentially, we will have new ontology concept classes created dynamically, if their types inherit
  from `owlready2.Thing`, all custom non-semantic (of types known only by PyCram) attributes, which are defined by their
  own in child classes, will apparently be not savable into the ontology by `owlready2` api. Then the next time the
  ontology is loaded, those same dynamic classes will not be created anymore, thus without those attributes either,
  causing running error.

As such, in short, an ontology concept class, either newly created on the fly or loaded from ontologies, has to
be `owlready2.Thing` or its pure derived class (without non-semantic attributes), so to make itself reusable upon
reloading.

Notable attributes:

- `ontology_concept`: An ontology concept of `owlready2.Thing` type or its pure child class (without custom non-semantic
  attributes), either dynamically created, or loaded from an ontology

- `designators`: a list of `DesignatorDescription` instances associated with `ontology_concept`

- `resolve`: a `Callable` typically returning a list of `DesignatorDescription` as specific designators,
  like `designators` or its subset, inferred from the ontology concept. In fact, it can be resolved to anything else
  relevant, up to the caller.

<!-- #endregion -->

<!-- #region jupyter={"outputs_hidden": false} -->

## Query ontology classes and their properties

Classes in the loaded ontology can be queried based on their exact names, or part of them, or by namespace.
Here, we can see essential info (ancestors, super/sub-classes, properties, direct instances, etc.) of the found ontology
class.
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
ontology_designed_container_class = ontology_manager.get_ontology_class('DesignedContainer')
ontology_manager.print_ontology_class(ontology_designed_container_class)
classes = ontology_manager.get_ontology_classes_by_subname('PhysicalObject');
print(classes[0])
classes = ontology_manager.get_ontology_classes_by_namespace('SOMA');
print(classes[:2])
```

<!-- #region jupyter={"outputs_hidden": false} -->
__Descendants__ of an ontology class can be also queried by
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
ontology_manager.get_ontology_descendant_classes(ontology_designed_container_class)[:5]
```

<!-- #region jupyter={"outputs_hidden": false} -->

## Create a new ontology concept class and its individual

A new ontology class can be created dynamically as inheriting from an existing class in the loaded ontology.
Here we create the class and its instance, also known as [__individual
__](https://owlready2.readthedocs.io/en/latest/class.html#creating-equivalent-classes) in ontology terms, which is then
wrapped inside an {class}`~pycram.ontology.ontology_common.OntologyConceptHolder`.
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
ontology_custom_container_class = ontology_manager.create_ontology_concept_class('CustomContainerConcept',
                                                                                 ontology_designed_container_class)
custom_container_concept_holder = OntologyConceptHolder(
    ontology_custom_container_class(name='ontology_custom_container_concept',
                                    namespace=main_ontology))
```

<!-- #region jupyter={"outputs_hidden": false} -->

## Access ontology concept classes and individuals

All ontology classes created on the fly purely inherit (without added non-semantic attributes) from `owlready2.Thing`,
and so share the same namespace with the loaded ontology instance, `main_ontology`. They can then be accessible through
that namespace by __main_ontology.<class_name>__.
The same applies for individuals of those classes, accessible by __main_ontology.<class_individual_name>__
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
ontology_manager.print_ontology_class(main_ontology.CustomContainerConcept)
print(
    f"custom_container_concept is {main_ontology.ontology_custom_container_concept}: {custom_container_concept_holder.ontology_concept is main_ontology.ontology_custom_container_concept}")
```

<!-- #region jupyter={"outputs_hidden": false} -->
For ones already existing in the ontology, they can only be accessed through their corresponding ontology, eg: `soma` as
follows
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
ontology_manager.print_ontology_class(soma.Cup)
```

<!-- #region jupyter={"outputs_hidden": false} -->

## Connect ontology class individuals with designators

After creating `custom_container_concept_holder` (wrapping `custom_container_concept` as an `owlready2.Thing`), we
connect it to a designator (say `obj_designator`) by:

- Appending to `obj_designator.ontology_concept_holders` with `custom_container_concept_holder`

- Appending to `custom_container_concept_holder.designators` with `obj_designator`

<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
custom_container_designator = ObjectDesignatorDescription(names=["obj"])
custom_container_designator.ontology_concept_holders.append(custom_container_concept_holder)
custom_container_concept_holder.designators.append(custom_container_designator)
```

<!-- #region jupyter={"outputs_hidden": false} -->
We can also automatize all the above setup with a single function call
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
another_custom_container_designator = ontology_manager.create_ontology_linked_designator(
    object_name="another_custom_container",
    designator_class=ObjectDesignatorDescription,
    ontology_concept_name="AnotherCustomContainerConcept",
    ontology_parent_class=ontology_designed_container_class)
another_custom_container_concept = another_custom_container_designator.ontology_concept_holders[0].ontology_concept
print(f"Ontology concept: {another_custom_container_concept.name} of class {type(another_custom_container_concept)}")
another_custom_container_designator = OntologyConceptHolderStore().get_ontology_concept_holder_by_name(
    main_ontology.AnotherCustomContainerConcept.instances()[0].name).get_default_designator()
print(f"Designator: {another_custom_container_designator.names[0]} of type {type(another_custom_container_designator)}")
```

<!-- #region jupyter={"outputs_hidden": false} -->

## Create new ontology triple classes

Concept classes of a triple, aka [__subject, predicate, object__], can be created dynamically. Here we will make an
example creating ones for [__handheld objects__] and [__placeholder objects__], with a pair of predicate and inverse
predicate signifying their mutual relation.
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
PLACEABLE_ON_PREDICATE_NAME = "placeable_on"
HOLD_OBJ_PREDICATE_NAME = "hold_obj"
ontology_manager.create_ontology_triple_classes(ontology_subject_parent_class=soma.DesignedContainer,
                                                subject_class_name="OntologyPlaceHolderObject",
                                                ontology_object_parent_class=soma.Shape,
                                                object_class_name="OntologyHandheldObject",
                                                predicate_class_name=PLACEABLE_ON_PREDICATE_NAME,
                                                inverse_predicate_class_name=HOLD_OBJ_PREDICATE_NAME,
                                                ontology_property_parent_class=soma.affordsBearer,
                                                ontology_inverse_property_parent_class=soma.isBearerAffordedBy)
ontology_manager.print_ontology_property(main_ontology.placeable_on)
ontology_manager.print_ontology_property(main_ontology.hold_obj)
```

<!-- #region jupyter={"outputs_hidden": false} -->
There, we use `soma.DesignedContainer` & `soma.Shape`, existing concept in SOMA ontology, as the parent classes for the
subject & object concepts respectively.
There is also a note that those classes will have the same namespace with `main_ontology`, so later on to be accessible
through it.

Then now we define some instances of the newly created triple classes, and link them to object designators, again
using `ontology_manager.create_ontology_linked_designator()`
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
def create_ontology_handheld_object_designator(object_name: str, ontology_parent_class: Type[owlready2.Thing]):
    return ontology_manager.create_ontology_linked_designator(object_name=object_name,
                                                              designator_class=ObjectDesignatorDescription,
                                                              ontology_concept_name=f"Onto{object_name}",
                                                              ontology_parent_class=ontology_parent_class)


# Holdable Objects
cookie_box = create_ontology_handheld_object_designator("cookie_box", main_ontology.OntologyHandheldObject)
egg = create_ontology_handheld_object_designator("egg", main_ontology.OntologyHandheldObject)

# Placeholder objects
placeholders = [create_ontology_handheld_object_designator(object_name, main_ontology.OntologyPlaceHolderObject)
                for object_name in ['table', 'stool', 'shelf']]

egg_tray = create_ontology_handheld_object_designator("egg_tray", main_ontology.OntologyPlaceHolderObject)
```

<!-- #region jupyter={"outputs_hidden": false} -->

### Create ontology relations

Now we will create ontology relations or predicates between __placeholder objects__ and __handheld objects__
with `ontology_manager.set_ontology_relation()`
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
for place_holder in placeholders:
    ontology_manager.set_ontology_relation(subject_designator=cookie_box, object_designator=place_holder,
                                           predicate_name=PLACEABLE_ON_PREDICATE_NAME)

ontology_manager.set_ontology_relation(subject_designator=egg_tray, object_designator=egg,
                                       predicate_name=HOLD_OBJ_PREDICATE_NAME)
```

<!-- #region jupyter={"outputs_hidden": false} -->

## Query designators based on their ontology-concept relations

Now we can make queries for designators from designators, based on the relation among their corresponding ontology
concepts setup above
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
print(f"{cookie_box.names}'s placeholder candidates:",
      f"""{[placeholder.names for placeholder in
            ontology_manager.get_designators_by_subject_predicate(subject=cookie_box,
                                                                  predicate_name=PLACEABLE_ON_PREDICATE_NAME)]}""")

print(f"{egg.names}'s placeholder candidates:",
      f"""{[placeholder.names for placeholder in
            ontology_manager.get_designators_by_subject_predicate(subject=egg,
                                                                  predicate_name=PLACEABLE_ON_PREDICATE_NAME)]}""")

for place_holder in placeholders:
    print(f"{place_holder.names} can hold:",
          f"""{[placeholder.names for placeholder in
                ontology_manager.get_designators_by_subject_predicate(subject=place_holder,
                                                                      predicate_name=HOLD_OBJ_PREDICATE_NAME)]}""")

print(f"{egg_tray.names} can hold:",
      f"""{[placeholder.names for placeholder in
            ontology_manager.get_designators_by_subject_predicate(subject=egg_tray,
                                                                  predicate_name=HOLD_OBJ_PREDICATE_NAME)]}""")
```

<!-- #region jupyter={"outputs_hidden": false} -->

# Practical examples

## Example 1

How about creating ontology concept classes encapsulating {class}`pycram.datastructures.enums.ObjectType`? We can do it by:
<!-- #endregion -->

```python jupyter={"outputs_hidden": false}
from pycram.datastructures.enums import ObjectType

# Create a generic ontology concept class for edible objects
generic_edible_class = ontology_manager.create_ontology_concept_class('GenericEdible')

# Create a list of object designators sharing the same concept class as [generic_edible_class]
edible_obj_types = [ObjectType.MILK, ObjectType.BREAKFAST_CEREAL]
for object_type in ObjectType:
    if object_type in edible_obj_types:
        # Create a designator for the edible object
        ontology_manager.create_ontology_object_designator_from_type(object_type, generic_edible_class)

print(f'{generic_edible_class.name} object types:')
for edible_ontology_concept in generic_edible_class.direct_instances():
    print(edible_ontology_concept,
          [des.types for des in
           OntologyConceptHolderStore().get_ontology_concept_holder_by_name(edible_ontology_concept.name).designators])

```

<!-- #region jupyter={"outputs_hidden": false} -->

## Example 2

We could also make use of relations between ontology concepts that designators are associated with, to enable more
abstract inputs in robot motion plans.

In a similar style to the scenario of __placeholder objects__ and __handheld objects__ above, but with a bit difference,
we will ask the robot to query which content holders (eg. cup, pitcher, bowl) whereby a milk box could be pourable into.

Basically, we will provide an ontology-based implementation for the query:

`abstract_ontology_concept -> specific_objects_in_world?`

To achieve it, we will create triple classes and configure a customized `resolve()` for the abstract concept, which
returns its associated specific designators.
These designators are then used to again resolve for the target objects of interest, which become the inputs to a robot
motion plan.

### Setup simulated environment

<!-- #endregion -->

```python
from pycram.worlds.bullet_world import BulletWorld, Object
from pycram.datastructures.pose import Pose

from pycram.process_module import simulated_robot
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *

world = BulletWorld()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
pr2 = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
kitchen_designator = ObjectDesignatorDescription(names=["kitchen"])
robot_designator = ObjectDesignatorDescription(names=["pr2"]).resolve()
```

### Create PourableObject-LiquidHolder triple ontology classes

```python
POURABLE_INTO_PREDICATE_NAME = "pourable_into"
HOLD_LIQUID_PREDICATE_NAME = "hold_liquid"
ontology_manager.create_ontology_triple_classes(ontology_subject_parent_class=soma.DesignedContainer,
                                                subject_class_name="OntologyLiquidHolderObject",
                                                ontology_object_parent_class=soma.Shape,
                                                object_class_name="OntologyPourableObject",
                                                predicate_class_name=POURABLE_INTO_PREDICATE_NAME,
                                                inverse_predicate_class_name=HOLD_LIQUID_PREDICATE_NAME,
                                                ontology_property_parent_class=soma.affordsBearer,
                                                ontology_inverse_property_parent_class=soma.isBearerAffordedBy)
```

### Spawn a pourable object & liquid holders into the world and Create their designators

```python
# Holdable obj
milk_box = Object("milk_box", ObjectType.MILK, "milk.stl")
milk_box_designator = create_ontology_handheld_object_designator(milk_box.name, main_ontology.OntologyPourableObject)

# Liquid-holders
cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=Pose([1.4, 1, 0.9]))
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([1.4, 0.5, 0.9]))
pitcher = Object("pitcher", ObjectType.GENERIC_OBJECT, "Static_MilkPitcher.stl", pose=Pose([1.4, 0, 0.9]))
milk_holders = [cup, bowl, pitcher]
milk_holder_designators = [
    create_ontology_handheld_object_designator(obj.name, main_ontology.OntologyLiquidHolderObject)
    for obj in milk_holders]
```

### Create an ontology relation between the designators of the pourable object & its liquid holders

```python
for milk_holder_desig in milk_holder_designators:
    ontology_manager.set_ontology_relation(subject_designator=milk_box_designator, object_designator=milk_holder_desig,
                                           predicate_name=POURABLE_INTO_PREDICATE_NAME)
```

### Set up `resolve` for the ontology concept of the pourable object

```python
milk_box_concept_holder = milk_box_designator.ontology_concept_holders[0]


def milk_box_concept_resolve():
    object_designator = ontology_manager.get_designators_by_subject_predicate(subject=milk_box_designator,
                                                                              predicate_name=POURABLE_INTO_PREDICATE_NAME)[
        0]
    return object_designator, object_designator.resolve()


milk_box_concept_holder.resolve = milk_box_concept_resolve
```

Here, for demonstration purpose only, we specify the resolving result by `milk_box_concept_holder` as `cup`, the
first-registered (default) pourable-into target milk holder, utilizing the ontology relation setup above.

Now, we can query the milk box's target liquid holder by resolving `milk_box_concept_holder`

```python
target_milk_holder_designator, target_milk_holder = milk_box_concept_holder.resolve()
print(
    f"Pickup target object: {target_milk_holder.name}, a content holder for {milk_box_designator.names} as in relation `{POURABLE_INTO_PREDICATE_NAME}`")
```

### Robot picks up the target liquid holder

```python
with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.3]).resolve().perform()

    pickup_pose = CostmapLocation(target=target_milk_holder, reachable_for=robot_designator).resolve()
    pickup_arm = pickup_pose.reachable_arms[0]

    print(pickup_pose, pickup_arm)

    NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()

    PickUpAction(object_designator_description=target_milk_holder_designator, arms=[pickup_arm],
                 grasps=[Grasp.FRONT]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    place_island = SemanticCostmapLocation("kitchen_island_surface", kitchen_designator.resolve(),
                                           target_milk_holder_designator.resolve()).resolve()

    place_stand = CostmapLocation(place_island.pose, reachable_for=robot_designator, reachable_arm=pickup_arm).resolve()

    NavigateAction(target_locations=[place_stand.pose]).resolve().perform()

    PlaceAction(target_milk_holder_designator, target_locations=[place_island.pose],
                arms=[pickup_arm]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
world.exit()
```

# Save ontologies to an OWL file

After all the above operations on our ontologies, we now can save them to an OWL file on disk

```python
ontology_manager.save(f"{Path.home()}/ontologies/New{main_ontology.name}.owl")
```

# Optimize ontology loading with SQLite3

Upon the initial ontology loading from OWL, an SQLite3 file is automatically created, acting as the quadstore cache for
the loaded ontologies. This allows them to be __selectively__ reusable the next time being loaded.
More info can be referenced [here](https://owlready2.readthedocs.io/en/latest/world.html). 
