from __future__ import annotations

import dataclasses
from typing_extensions import List, Optional, Callable, TYPE_CHECKING
import sqlalchemy.orm
from ..datastructures.world import World
from ..world_concepts.world_object import Object as WorldObject
from ..designator import ObjectDesignatorDescription
from ..orm.base import ProcessMetaData
from ..orm.object_designator import (BelieveObject as ORMBelieveObject, ObjectPart as ORMObjectPart)
from ..datastructures.pose import Pose
from ..external_interfaces.robokudo import query

if TYPE_CHECKING:
    import owlready2

class BelieveObject(ObjectDesignatorDescription):
    """
    Description for Objects that are only believed in.
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        """
        Concrete object that is believed in.
        """

        def to_sql(self) -> ORMBelieveObject:
            return ORMBelieveObject(self.obj_type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMBelieveObject:
            metadata = ProcessMetaData().insert(session)
            self_ = self.to_sql()
            self_.process_metadata = metadata
            session.add(self_)

            return self_


class ObjectPart(ObjectDesignatorDescription):
    """
    Object Designator Descriptions for Objects that are part of some other object.
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):

        # The rest of attributes is inherited
        part_pose: Pose

        def to_sql(self) -> ORMObjectPart:
            return ORMObjectPart(self.obj_type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectPart:
            metadata = ProcessMetaData().insert(session)
            pose = self.part_pose.insert(session)
            obj = self.to_sql()
            obj.process_metadata = metadata
            obj.pose = pose
            session.add(obj)

            return obj

    def __init__(self, names: List[str],
                 part_of: ObjectDesignatorDescription.Object,
                 type: Optional[str] = None,
                 resolver: Optional[Callable] = None):
        """
        Describing the relationship between an object and a specific part of it.

        :param names: Possible names for the part
        :param part_of: Parent object of which the part should be described
        :param type: Type of the part
        :param resolver: An alternative specialized_designators to resolve the input parameter to an object designator
        :param ontology_concept_holders: A list of ontology concepts that the object part is categorized as or associated with
        """
        super().__init__(names, type, resolver)

        if not part_of:
            raise AttributeError("part_of cannot be None.")

        self.type: Optional[str] = type
        self.names: Optional[List[str]] = names
        self.part_of = part_of

    def ground(self) -> Object:
        """
        Default specialized_designators, returns the first result of the iterator of this instance.

        :return: A resolved object designator
        """
        return next(iter(self))

    def __iter__(self):
        """
        Iterates through every possible solution for the given input parameter.

        :yield: A resolved Object designator
        """
        for name in self.names:
            if name in self.part_of.world_object.link_name_to_id.keys():
                yield self.Object(name, self.type, self.part_of.world_object,
                                  self.part_of.world_object.get_link_pose(name))


class LocatedObject(ObjectDesignatorDescription):
    """
    Description for KnowRob located objects.
    **Currently has no specialized_designators**
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        reference_frame: str
        """
        Reference frame in which the position is given
        """
        timestamp: float
        """
        Timestamp at which the position was valid
        """

    def __init__(self, names: List[str], types: List[str],
                 reference_frames: List[str], timestamps: List[float], resolver: Optional[Callable] = None,
                 ontology_concept_holders: Optional[List[owlready2.Thing]] = None):
        """
        Describing an object resolved through knowrob.

        :param names: List of possible names describing the object
        :param types: List of possible types describing the object
        :param reference_frames: Frame of reference in which the object position should be
        :param timestamps: Timestamps for which positions should be returned
        :param resolver: An alternative specialized_designators that resolves the input parameter to an object designator.
        :param ontology_concept_holders: A list of ontology concepts that the object is categorized as
        """
        super(LocatedObject, self).__init__(names, types, resolver, ontology_concept_holders)
        self.reference_frames: List[str] = reference_frames
        self.timestamps: List[float] = timestamps


class RealObject(ObjectDesignatorDescription):
    """
    Object designator representing an object in the real world, when resolving this object designator description ]
    RoboKudo is queried to perceive an object fitting the given criteria. Afterward the specialized_designators tries to match
    the found object to an Object in the World.
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        pose: Pose
        """
        Pose of the perceived object
        """

    def __init__(self, names: Optional[List[str]] = None, types: Optional[List[str]] = None,
                 world_object: WorldObject = None, resolver: Optional[Callable] = None):
        """
        
        :param names: 
        :param types: 
        :param world_object:
        :param resolver: 
        """
        super().__init__(resolver)
        self.types: Optional[List[str]] = types
        self.names: Optional[List[str]] = names
        self.world_object: WorldObject = world_object

    def __iter__(self):
        """
        Queries RoboKudo for objects that fit the description and then iterates over all World objects that have
        the same type to match a World object to the real object.

        :yield: A resolved object designator with reference world object
        """
        object_candidates = query(self)
        for obj_desig in object_candidates:
            for world_obj in World.get_object_by_type(obj_desig.obj_type):
                obj_desig.world_object = world_obj
                yield obj_desig
