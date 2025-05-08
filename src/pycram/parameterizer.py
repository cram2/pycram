from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Any, Type

from random_events.product_algebra import Event
from random_events.set import Set
from random_events.variable import Symbolic, Integer, Variable, Continuous

from .plan import Plan, DesignatorNode


@dataclass
class Parameterizer:
    plan: Plan

    parameters: Dict[DesignatorNode, Any] = field(default_factory=dict, init=False)

    event: Event = field(init=False, default=None)

    def __post_init__(self):
        self.make_parameters()
        self.make_event()

    def make_parameters(self):
        for node in self.plan.nodes:
            if isinstance(node, DesignatorNode):
                self.parameters[node] = node.action._parameters

    def make_event(self):
        variables = []
        for node, params in self.parameters.items():
            for name, leaf_type in node.flattened_parameters().items():
                variables.append(leaf_type_to_variable(name, leaf_type))
        print(variables)

def leaf_type_to_variable(name: str, leaf_type: Type) -> Variable:
    print(leaf_type)
    if issubclass(leaf_type, bool):
        return Symbolic(name, Set.from_iterable([True, False]))
    elif issubclass(leaf_type, Enum):
        return Symbolic(name, Set.from_iterable(leaf_type))
    elif issubclass(leaf_type, int):
        return Integer(name)
    elif issubclass(leaf_type, float):
        return Continuous(name)
    else:
        raise NotImplementedError(f"No conversion between {leaf_type} and random_events.Variable is known.")
