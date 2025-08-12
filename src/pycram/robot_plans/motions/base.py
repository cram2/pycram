from __future__ import annotations

import inspect
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from semantic_world.world import World
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from ...plan import PlanNode, Plan


@dataclass
class BaseMotion(ABC):

    # Is assigned in the __post_init method of the MotionNode
    plan_node: PlanNode = field(init=False, default=None)

    @property
    def plan(self) -> Plan:
        return self.plan_node.plan

    @property
    def world(self):
        return self.plan.world

    @abstractmethod
    def perform(self):
        """
        Passes this designator to the process module for execution. Will be overwritten by each motion.
        """
        pass

    def __post_init__(self):
        """
        Checks if types are missing or wrong
        """

        return
        # TODO include type checks for this again (use type guard?)
        #
        # right_types = get_type_hints(self)
        # attributes = self.__dict__.copy()
        #`
        # missing = []
        # wrong_type = {}
        # current_type = {}
        #
        # for k in attributes.keys():
        #     attribute = attributes[k]
        #     attribute_type = type(attributes[k])
        #     right_type = right_types[k]
        #     types = get_args(right_type)
        #     if attribute is None:
        #         if not any([x is type(None) for x in get_args(right_type)]):
        #             missing.append(k)
        #     elif not issubclass(attribute_type, right_type): # attribute_type is not right_type:
        #         if attribute_type not in types:
        #             if attribute_type not in [get_origin(x) for x in types if x is not type(None)]:
        #                 wrong_type[k] = right_types[k]
        #                 current_type[k] = attribute_type
        # if missing != [] or wrong_type != {}:
        #     raise ResolutionError(missing, wrong_type, current_type, self.__class__)
        #
