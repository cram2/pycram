import dataclasses
from typing import List, Union, Optional, Callable

import sqlalchemy.orm

from ..bullet_world import BulletWorld, Object as BulletWorldObject
from ..designator import DesignatorDescription
from ..orm.base import (Position as ORMPosition, Quaternion as ORMQuaternion)
from ..orm.object_designator import (ObjectDesignator as ORMObjectDesignator,
                                     BelieveObject as ORMBelieveObject,
                                     ObjectPart as ORMObjectPart)


class ObjectDesignatorDescription(DesignatorDescription):
    """
    Class for object designator descriptions.
    Descriptions hold possible parameter ranges for object designators.
    :ivar types: Types to consider
    :ivar names: Names to consider
    """

    @dataclasses.dataclass
    class Object:
        """
        A single element that fits the description.
        """
        name: str
        type: str
        # pose:
        # pose: Tuple[List[float], List[float]]
        bullet_world_object: Optional[BulletWorldObject]

        def to_sql(self) -> ORMObjectDesignator:
            return ORMObjectDesignator(self.type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectDesignator:
            obj = self.to_sql()
            if self.pose:
                position = ORMPosition(*self.pose[0])
                orientation = ORMQuaternion(*self.pose[1])
                session.add(position)
                session.add(orientation)
                session.commit()
                obj.position = position.id
                obj.orientation = orientation.id
            else:
                obj.position = None
                obj.orientation = None

            session.add(obj)
            session.commit()
            return obj

        @property
        def pose(self):
            return self.bullet_world_object.get_position_and_orientation()

        def __repr__(self):
            return self.__class__.__qualname__ + f"(" + ', '.join([f"{f.name}={self.__getattribute__(f.name)}"
                                                                   for f in dataclasses.fields(self)] +
                                                                  [f"pose={self.pose}"]) + ')'

    def __init__(self, names: Optional[List[str]] = None,
                 types: Optional[List[str]] = None,
                 resolver: Optional[Callable] = None):
        super().__init__(resolver)
        self.types: Optional[List[str]] = types
        self.names: Optional[List[str]] = names

    def ground(self) -> Union[Object, bool]:
        """Return the first object from the bullet world that fits the description."""
        return next(iter(self))

    def __iter__(self) -> Object:
        """
        Iterate through all possible objects fitting this description
        """
        # for every bullet world object
        for obj in BulletWorld.current_bullet_world.objects:

            # skip if name does not match specification
            if self.names and obj.name not in self.names:
                continue

            # skip if type does not match specification
            if self.types and obj.type not in self.types:
                continue

            yield self.Object(obj.name, obj.type, obj)


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
            return self_


class ObjectPart(ObjectDesignatorDescription):
    """
    Object Designator Descriptions for Objects that are part of some other object.

    :ivar part_of: The description of potential objects this is part of
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):

        part_of: ObjectDesignatorDescription.Object

        def to_sql(self) -> ORMObjectPart:
            return ORMObjectPart(self.type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectPart:
            obj = self.to_sql()

            # try to create the part_of object
            if self.part_of:
                part = self.part_of.insert(session)
                obj.part_of = part.id
            else:
                obj.part_of = None

            session.add(obj)
            session.commit()

            return obj

    def __init__(self, names: Optional[List[str]] = None,
                 types: Optional[List[str]] = None,
                 part_of: Optional[ObjectDesignatorDescription] = None,
                 resolver: Optional[Callable] = None):
        super().__init__(names, types, resolver)

        if not part_of:
            raise AttributeError("part_of cannot be None.")

        self.types: Optional[List[str]] = types
        self.names: Optional[List[str]] = names
        self.part_of = part_of

    def ground(self) -> Object:
        return next(iter(self))

    def __iter__(self):
        for part_of_obj in iter(self.part_of):

            if self.names:
                for name in self.names:
                    if name in part_of_obj.bullet_world_object.links.keys():
                        yield self.Object(name, "",
                                          part_of_obj.bullet_world_object.get_link_position_and_orientation(name),
                                          None,
                                          part_of_obj)


class LocatedObject(ObjectDesignatorDescription):
    """
    Description for KnowRob located objects.

    :ivar timestamps: timestamps to consider?
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        reference_frame: str
        timestamp: float

    def __init__(self, names: List[str], types: List[str],
                 reference_frames: List[str], timestamps: List[float], resolver: Optional[Callable] = None):
        super(LocatedObject, self).__init__(names, types, resolver)
        self.reference_frames: List[str] = reference_frames
        self.timestamps: List[float] = timestamps
