from __future__ import annotations

import dataclasses

import owlready2
from owlready2.triplelite import _SearchList
from semantic_world.world_entity import Body
from typing_extensions import TYPE_CHECKING, Iterable, Iterator, Union

from ..datastructures.enums import ObjectType
from ..datastructures.partial_designator import PartialDesignator
from ..external_interfaces.robokudo import *
from ..plan import Plan
from ..utils import is_iterable


if TYPE_CHECKING:
    pass


class OntologyObjectDesignatorDescription:
    """
    Description for Objects that can be found using ontological reasoning
    """

    search_result: List
    """
    The result from the search in the ontology.
    """

    def __init__(self, search_result: _SearchList):
        self.search_result = list(search_result)

    def __iter__(self) -> Iterable[Body]:
        """
        :return: The objects in the current world which match the search result in the 'is_a' relation.
        """
        for obj in World.current_world.objects:

            # expand is_a of the object individual
            is_a = obj.ontology_individual.is_a + [obj.ontology_individual]

            # get the matching concepts
            intersection = ([x for x in is_a if x in self.search_result])

            # if it matches
            if len(intersection) > 0:
                yield obj


class BelieveObject(ObjectDesignatorDescription):
    """
    Description for Objects that are only believed in.
    """

class ResolutionStrategyObject(ObjectDesignatorDescription):

    def __init__(self, strategy: Union[Callable, Iterable]):
        """
        Description for Objects that are only believed in.

        :param strategy: The strategy to use for the resolution
        """
        super().__init__()
        self.strategy = strategy

    def create_iterator(self, resolution_strategy: Union[Callable, Iterable]):
        """
        Creates an iterator for the given method. If the method is iterable it will be used as is, otherwise it will
        be called as a function.

        :param resolution_strategy: The method to create an iterator for.
        :return: An iterator for the given method.
        """

        class IterClass:
            def __init__(self, method: Union[Callable, Iterable]):
                self.method = method

            def __iter__(self):
                if callable(self.method):
                    yield self.method()
                elif is_iterable(self.method()):
                    for i in self.method():
                        yield i

        if isinstance(resolution_strategy, Plan):
            resolution_strategy = resolution_strategy.perform
        resolution_strategy = IterClass(resolution_strategy)
        return resolution_strategy

    def __iter__(self) -> Iterable[Body]:
        """
        Iterates through every possible solution for the given solution strategy.

        :return: A resolved object designator
        """
        for obj in self.create_iterator(self.strategy):
            yield obj