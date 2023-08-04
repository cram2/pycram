import dataclasses
from typing import List, Union, Optional, Callable, Tuple, Iterable
import sqlalchemy.orm

from ..bullet_world import BulletWorld, Object as BulletWorldObject
from ..designator import DesignatorDescription, ObjectDesignatorDescription
from ..orm.base import (Position as ORMPosition, Quaternion as ORMQuaternion, MetaData)
from ..orm.object_designator import (ObjectDesignator as ORMObjectDesignator, BelieveObject as ORMBelieveObject,
                                     ObjectPart as ORMObjectPart)
from ..pose import Pose


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
            return ORMBelieveObject(self.type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMBelieveObject:
            self_ = self.to_sql()
            session.add(self_)
            session.commit()
            metadata = MetaData().insert(session)
            self_.metadata_id = metadata.id
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
            return ORMObjectPart(self.type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectPart:
            obj = self.to_sql()
            metadata = MetaData().insert(session)
            obj.metadata_id = metadata.id
            # try to create the part_of object
            if self.part_of:
                part = self.part_of.insert(session)
                obj.part_of = part.id
            else:
                obj.part_of = None

            session.add(obj)
            session.commit()

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
        :param resolver: An alternative resolver to resolve the input parameter to an object designator
        """
        super().__init__(names, type, resolver)

        if not part_of:
            raise AttributeError("part_of cannot be None.")

        self.type: Optional[str] = type
        self.names: Optional[List[str]] = names
        self.part_of = part_of

    def ground(self) -> Object:
        """
        Default resolver, returns the first result of the iterator of this instance.

        :return: A resolved object designator
        """
        return next(iter(self))

    def __iter__(self):
        """
        Iterates through every possible solution for the given input parameter.

        :yield: A resolved Object designator
        """
        for name in self.names:
            if name in self.part_of.bullet_world_object.links.keys():
                yield self.Object(name, self.type, self.part_of.bullet_world_object,
                                  self.part_of.bullet_world_object.get_link_pose(name))


class LocatedObject(ObjectDesignatorDescription):
    """
    Description for KnowRob located objects.
    **Currently has no resolver**
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
                 reference_frames: List[str], timestamps: List[float], resolver: Optional[Callable] = None):
        """
        Describing an object resolved through knowrob.

        :param names: List of possible names describing the object
        :param types: List of possible types describing the object
        :param reference_frames: Frame of reference in which the object position should be
        :param timestamps: Timestamps for which positions should be returned
        :param resolver: An alternative resolver that resolves the input parameter to an object designator.
        """
        super(LocatedObject, self).__init__(names, types, resolver)
        self.reference_frames: List[str] = reference_frames
        self.timestamps: List[float] = timestamps
