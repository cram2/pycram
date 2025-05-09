import copy
import itertools
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Any, Type, List

import numpy as np
from probabilistic_model.probabilistic_model import ProbabilisticModel
from random_events.product_algebra import Event, SimpleEvent
from random_events.set import Set
from random_events.variable import Symbolic, Integer, Variable, Continuous

from .datastructures.partial_designator import PartialDesignator
from .language import SequentialPlan
from .plan import Plan, DesignatorNode, ActionNode


@dataclass
class Parameterizer:
    plan: Plan

    parameters: Dict[DesignatorNode, Any] = field(default_factory=dict, init=False)

    event: Event = field(init=False, default=None)

    variables_of_node: Dict[DesignatorNode, List[Variable]] = field(default_factory=dict, init=False)

    def __post_init__(self):
        self.make_parameters()
        self.make_variables()

    @property
    def variables(self):
        return list(itertools.chain.from_iterable(self.variables_of_node.values()))

    def get_variable(self, name: str) -> Variable:
        return [v for v  in self.variables if v.name == name][0]

    def make_parameters(self):
        for node in self.plan.nodes:
            if isinstance(node, DesignatorNode):
                self.parameters[node] = node.action._parameters

    def make_variables(self):
        for index, (node, params) in enumerate(self.parameters.items()):
            variables = []
            for name, leaf_type in node.flattened_parameters().items():
                full_name = f"{node.action.__name__}_{index}.{name}"
                variable = leaf_type_to_variable(full_name, leaf_type)
                variables.append(variable)
            self.variables_of_node[node] = variables


    def plan_from_sample(self, model: ProbabilisticModel, sample: np.array) -> Plan:

        sub_plans = []

        for node in self.variables_of_node:
            flattened_parameters = []
            for variable in self.variables_of_node[node]:
                flattened_parameters.append(sample[model.variables.index(variable)])
            resolved = node.action.reconstruct(flattened_parameters)

            kwargs = {key: getattr(resolved, key) for key in resolved._parameters}
            sub_plans.append(Plan(ActionNode(designator_ref=PartialDesignator(node.action, **kwargs),
                       action = node.action, kwargs=kwargs)))


        return SequentialPlan(*sub_plans)





def leaf_type_to_variable(name: str, leaf_type: Type) -> Variable:
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
