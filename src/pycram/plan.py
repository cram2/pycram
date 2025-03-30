from __future__ import annotations

from dataclasses import field, dataclass
from datetime import datetime


import networkx as nx


from typing_extensions import Optional, Callable, Any, Dict, List, Self

from .datastructures.enums import TaskStatus
from pycrap.ontologies import Action
from .datastructures.partial_designator import PartialDesignator
from .failures import PlanFailure


class Plan(nx.DiGraph):
    """
    Represents a plan structure, typically a tree, which can be changed at any point in time. Performing the plan will
    traverse the plan structure in depth first order and perform each PlanNode
    """
    current_plan: Plan = None

    def __init__(self):
        super().__init__()
        self.root: PlanNode = PlanNode(plan=self)
        self.add_node(self.root)
        self.current_node: PlanNode = self.root

    def mount(self, other: Plan, mount_node: PlanNode = None):
        mount_node = mount_node or self.root
        self.add_nodes_from(other.nodes)
        self.add_edges_from(other.edges)
        self.merge_nodes(mount_node, other.root)
        for node in self.nodes:
            node.plan = self

    def merge_nodes(self, node1: PlanNode, node2: PlanNode):
        for node in node2.children:
            self.add_edge(node1, node)
        self.remove_node(node2)

    def insert_node(self, node: PlanNode, path: str = None):
        if path:
            pass
        else:
            self.add_edge(self.root, node)
            node.plan = self

    def insert_below(self, insert_node: PlanNode, insert_below: PlanNode):
        self.add_edge(insert_below, insert_node)

    def insert_after(self, node: PlanNode):
        pass

    def find_node(self, designator) -> str:
        pass

    def find_plan(self, plan: Plan):
        plan_root = plan.root
        for node in self.nodes:
            if node == plan_root:
                return node

    def perform(self):
        pass

    def resolve(self):
        return self.root.children[0].designator_ref.resolve()

    def __enter__(self):
        self.prev_plan = Plan.current_plan
        Plan.current_plan = self

    def __exit__(self, exc_type, exc_val, exc_tb):
        Plan.current_plan = self.prev_plan


@dataclass
class PlanNode:
    status: TaskStatus = TaskStatus.CREATED
    """
    The status of the node from the TaskStatus enum.
    """

    start_time: Optional[datetime] = field(default_factory=datetime.now)
    """
    The starting time of the function, optional
    """

    end_time: Optional[datetime] = None
    """
    The ending time of the function, optional
    """

    reason: Optional[PlanFailure] = None
    """
    The reason of failure if the action failed.
    """

    plan: Plan = None
    """
    Reference to the plan to which this node belongs
    """

    @property
    def parent(self):
        return list(filter(None, [edge[0] if edge[1] == self else None for edge in  self.plan.edges]))

    @property
    def children(self):
        return list(filter(None, [edge[1] if edge[0] == self else None for edge in  self.plan.edges]))

    def __hash__(self):
        return id(self)


@dataclass
class ActionNode(PlanNode):
    designator_ref: PartialDesignator = None
    """
    Reference to the Designator in this node
    """

    action: Optional[Action] = None
    """
    The action and that is performed or None if nothing was performed
    """

    kwargs: Dict[str, Any] = None
    """
    kwargs of the action in this node
    """

    def __hash__(self):
        return id(self)

    def __repr__(self, *args, **kwargs):
        return f"<{self.designator_ref.performable.__name__}_{id(self)}>"


def with_tree(func: Callable) -> Callable:
    pass


def with_plan(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        action = func(*args, **kwargs)
        # TODO Maybe use bind_partial instead
        node = ActionNode(designator_ref=action, action="Action", kwargs=action.kwargs)
        plan = Plan()
        plan.insert_node(node)
        if Plan.current_plan:
            Plan.current_plan.mount(plan)
        return plan

    return wrapper


# from __future__ import annotations
import inspect
from dataclasses import dataclass, field
from functools import cached_property

import typing_extensions
from typing_extensions import Type, List, Dict, Any, Union, Tuple

atomic_types = (int, float, str, bool)

@dataclass
class ClassFlattener:
    """
    A utility class that handles flattening and reconstructing objects based on class type annotations.
    It extracts field types from class type hints and flattens nested objects into a list. It can also
    reconstruct the original object from a flattened list.
    """

    clazz: Type
    """
    The class type that is the target for flattening and reconstructing.
    """

    fields: Dict[str, Union[atomic_types, ClassFlattener]] = field(default_factory=dict)
    """
    A dictionary that maps field names to their types, including nested types.
    The keys are the names of the variables. The values are the types of the variables or nested types.
    """

    def __post_init__(self):
        # calculate the field
        for field_name, field_type in typing_extensions.get_type_hints(self.clazz.__init__).items():
            if field_name.startswith("_"):
                continue
            if field_type in atomic_types:
                self.fields[field_name] = field_type
            else:
                self.fields[field_name] = ClassFlattener(field_type)


    def flatten(self, obj) -> List:
        """
        Flattens the object into a list of field values.

        :param obj: The object to be flattened. It must be an instance of the target class.

        :return: A list of flattened field values from the object.

        :raises TypeError: If the object is not an instance of the target class.
        """
        if not isinstance(obj, self.clazz):
            raise TypeError("Object must be an instance of the target class")
        result = []
        for field_name, field_type in self.fields.items():
            if isinstance(field_type, ClassFlattener):
                sub_flattener = ClassFlattener(field_type.clazz)
                sub_obj = getattr(obj, field_name)
                result.extend(sub_flattener.flatten(sub_obj))
            else:
                result.append(getattr(obj, field_name))
        return result

    @cached_property
    def flattened_fields(self) -> Dict[str, atomic_types]:
        """
        Returns a dictionary of all flattened fields and their types.

        :return: A dictionary mapping field names to their types.
        """
        flat_fields = {}
        for field_name, field_type in self.fields.items():
            if isinstance(field_type, ClassFlattener):
                sub_flat_fields = field_type.flattened_fields
                for sub_field_name, sub_field_type in sub_flat_fields.items():
                    flat_fields[f"{field_name}.{sub_field_name}"] = sub_field_type
            else:
                flat_fields[field_name] = field_type
        return flat_fields

    @cached_property
    def number_of_fields(self) -> int:
        """
        :return: The total number of fields in the flattened list.
        """
        return len(self.flattened_fields)

    @cached_property
    def field_indices(self) -> Dict[str, Tuple[int, int]]:
        """
        :return: A dictionary mapping field names to their indices in the flattened list.
        """
        field_indices = {}
        index = 0
        for field_name, field_type in self.fields.items():
            if isinstance(field_type, ClassFlattener):
                field_indices[field_name] = (index, index + field_type.number_of_fields)
                index += field_type.number_of_fields
            else:
                field_indices[field_name] = (index, index + 1)
                index += 1
        return field_indices


    def reconstruct(self, flattened: List):
        """
        Reconstructs the object from a flattened list of field values.

        :param flattened: The flattened list of field values.

        :return: An instance of the target class with the reconstructed field values.

        :raises TypeError: If the object is not a list or if the length of the list does not match the number of fields.
        """
        if len(flattened) != len(self.flattened_fields):
            raise ValueError("List length does not match number of flattened fields")

        kwargs = {}
        for field_name, field_type in self.fields.items():
            start, end = self.field_indices[field_name]
            if isinstance(field_type, ClassFlattener):
                sub_obj = flattened[start:end]
                kwargs[field_name] = field_type.reconstruct(sub_obj)
            else:
                kwargs[field_name] = flattened[start]

        return self.clazz(**kwargs)

@dataclass
class NestedClass:
    nested_field: str
    another_nested_field: int

@dataclass
class MyClass:
    field1: int
    field2: str
    nested: NestedClass


def main():

    obj = MyClass(field1=42, field2="Hello", nested=NestedClass(nested_field="World", another_nested_field=100))
    flattener = ClassFlattener(MyClass)

    print(flattener.fields)
    print(flattener.flattened_fields)
    flattened = flattener.flatten(obj)
    print("Flattened:", flattened)
    print(flattener.field_indices)

    reconstructed = flattener.reconstruct(flattened)
    print("Reconstructed:", reconstructed)

if __name__ == "__main__":
    main()
