from __future__ import annotations

import dataclasses

import owlready2
import sqlalchemy.orm
from owlready2.triplelite import _SearchList
from typing_extensions import TYPE_CHECKING, Iterable, Iterator, Union

from ..datastructures.enums import ObjectType
from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.world import World
from ..external_interfaces.robokudo import *
from ..orm.base import ProcessMetaData
from ..orm.object_designator import (BelieveObject as ORMBelieveObject, ObjectPart as ORMObjectPart)
from ..world_concepts.world_object import Object as WorldObject, Object
from ..description import ObjectDescription

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

    def __iter__(self) -> Iterable[Object]:
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


class ObjectPart(ObjectDesignatorDescription, Iterable[ObjectDescription.Link]):
    """
    Object Designator Descriptions for Objects that are part of some other object.
    """

    def __init__(self, names: List[str],
                 part_of: Union[WorldObject, Iterable[WorldObject]],
                 type: Optional[ObjectType] = None):
        """
        Describing the relationship between an object and a specific part of it.

        :param names: Possible names for the part
        :param part_of: Parent object of which the part should be described
        :param type: Type of the part
        """
        super().__init__(names, type)
        PartialDesignator.__init__(self, ObjectPart, names=names, part_of=part_of, type=type)

        if not part_of:
            raise AttributeError("part_of cannot be None.")

        self.type: Optional[ObjectType] = type
        self.names: Optional[List[str]] = names
        self.part_of = part_of

    def ground(self) -> ObjectDescription.Link:
        """
        Default specialized_designators, returns the first result of the iterator of this instance.

        :return: A resolved object designator
        """
        return next(iter(self))

    def __iter__(self) -> Iterator[ObjectDescription.Link]:
        """
        Iterates through every possible solution for the given input parameter.

        :yield: A resolved Object designator
        """
        for params in self.generate_permutations():
            if params["names"] in params["part_of"].links.keys():
                yield params["part_of"].links[params["names"]]
