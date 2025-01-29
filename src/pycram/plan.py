from __future__ import annotations

from functools import reduce
from typing import List

import networkx as nx
from random_events.product_algebra import Event, SimpleEvent
from random_events.variable import *
from typing_extensions import Iterable, Tuple

from .datastructures.mixins import ParameterInfo
from .designators.action_designator import NavigateActionPerformable, ActionAbstract
from probabilistic_model.probabilistic_circuit.nx.helper import fully_factorized



class Boolean(SetElement):
    EMPTY_SET = -1
    TRUE = 1
    FALSE = 0


def path_and_value_of_nested_dict(nested_dict) -> Iterable[Tuple[List[str], Any]]:
    """
    Iterate over all paths and values of a nested dictionary.

    :param nested_dict: The nested dictionary to iterate over.
    :return: A generator of paths and values.
    """
    for key, value in nested_dict.items():
        if isinstance(value, dict):
            for path, value in path_and_value_of_nested_dict(value):
                yield [key] + path, value
        else:
            yield [key], value

class ActionCore:
    """
    A Node in a plan.
    A node is a description of an action that can be performed. This node can be used to extract all parameters that
    are possible to set for an action.
    """
    plan: Plan
    action_class: Type[ActionAbstract]

    def __init__(self, action_class: Type[ActionAbstract], plan: Optional[Plan] = None):
        self.action_class = action_class
        if plan is None:
            plan = Plan()
        plan.add_node(self)

    def __hash__(self):
        return id(self.action_class)

    def __repr__(self):
        return repr(self.action_class)

    @property
    def variables(self) -> List[Variable]:
        variables = []
        for path, value in path_and_value_of_nested_dict(self.action_class.parameters()):
            value: ParameterInfo
            name = self.action_class.__name__ + "." + ".".join(path) + "_" + str(hash(self))
            if issubclass(value.dtype, bool):
                variable = Symbolic(name, Boolean)
            elif issubclass(value.dtype, int):
                variable = Integer(name)
            elif issubclass(value.dtype, float):
                variable = Continuous(name)
            elif issubclass(value.dtype, SetElement):
                variable = Symbolic(name, value.dtype)
            else:
                raise ValueError(f"Unsupported type {value.dtype}")
            variable.path = path
            variable.action_core = self
            variables.append(variable)
        return variables

    def event(self):
        return SimpleEvent({variable: variable.domain for variable in self.variables}).as_composite_set()

    def parameters(self):
        return self.action_class.parameters()


class Plan(nx.DiGraph):

    def add_node(self, node: ActionCore, **attr):
        node.plan = self
        super().add_node(node, **attr)

    def add_nodes_from(self, nodes_for_adding: Iterable[ActionCore], **attr):
        for node in nodes_for_adding:
            self.add_node(node, **attr)

    def event(self):
        result = Event()
        for node in self.nodes:
            result |= node.event()
        return result

    def variables(self):
        return [variable for node in self.nodes for variable in node.variables]

    def parameters(self):
        return {node: node.parameters() for node in self.nodes}