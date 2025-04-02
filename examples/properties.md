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

# Property
Properties are the part of knowledge which is most prominent to the user. In this example we will go over what properties 
are, how they are defined and how they integrate into the bigger knowledge system. 

Properties are used in PyCRAM to annotate parameter of action designator with semantic properties. Properties are only 
proxy objects which do not implement the actual functionality of the property, this is done in the knowledge Source 
which can handle this property. For more information how Knowledge Sources work and how they interact with properties 
please look at the knowledge_source example. 

In this example we will define an example property and use this as a running example to walk through all steps involved 
in creating a new property. 

## Creating a new Property
To create a new property in PyCRAM we simply need to create a new class which inherits from the ```Property``` class.

```python
from pycram.datastructures.property import Property
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.dataclasses import ReasoningResult
from pycram.failures import ReasoningError
from abc import abstractmethod


class ExampleProperty(Property):
    property_exception = ReasoningError

    def __init__(self, pose: PoseStamped):
        super().__init__()
        self.pose = pose

    @abstractmethod
    def example(self, pose: PoseStamped) -> ReasoningResult:
        raise NotImplementedError
```

And with that we created a new property. While the process of creating a new property is simple and straight forward 
there are still a few things a user has to adhere to that should be kept in mind. 

* Only **one** method, besides the constructor, can defined otherwise the resolution will fail. 
* A ```property_exception``` has to be defined, which is raised in case the reasoning about the property is not 
* successful
* The reasoning method of the property can have an arbitraty amount of parameter, however, for each of these parameter 
* an instance variable with the same name has to be present. 
    * This means for this example: when the ```example``` method is called later on, while evaluating all properties, 
    * it will be called with the value of ```self.pose``` as parameter.  
    
Now we can use this property to annotate a variable. 

```python
example_property = ExampleProperty(Pose([1, 1, 1]))
```

As already mentioned do properties not implement the actual reasoning functionality and are just proxy objects to 
annotate parameter of action designator. The reasoning functionality is implemented in a Knowledge Source, so to test 
our new property we need to create a Knowledge Source which can handle the ```ExampleProperty```. For detailed 
information how a Knowledge Source is created pleas refere to the respective example.

## A Knowledge Source for our Property

```python
from pycram.knowledge.knowledge_source import KnowledgeSource
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.dataclasses import ReasoningResult


class ExampleKnowledge(KnowledgeSource, ExampleProperty):

    def __init__(self):
        super().__init__(name="example", priority=0)
        self.parameter = {}

    def is_available(self) -> bool:
        return True

    def is_connected(self) -> bool:
        return True

    def connect(self):
        pass

    def clear_state(self):
        self.parameter = {}

    def example(self, pose: PoseStamped) -> ReasoningResult:
        null_pose = PoseStamped.from_list([1, 1, 1])
        distance = null_pose.dist(pose)
        return ReasoningResult(distance == 0.0)

```

Since we now have a Knowledge Source that can handle our ```ExampleProperty``` we can resolve and evaluate the property. 
The reasoning implementation just checks if the distance between the given pose and a pose at [1, 1, 1] is zero, 
meaning they point at the same point. 

```python
from pycram.knowledge.knowledge_engine import KnowledgeEngine

ke = KnowledgeEngine()
resolved_property = ke.resolve_properties(example_property)

print(f"Result of the property: {resolved_property()}")
```
