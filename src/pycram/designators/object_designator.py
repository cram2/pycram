import dataclasses
from typing import List, Union, Optional, Callable, Tuple, Iterable
import sqlalchemy.orm

from ..bullet_world import BulletWorld, Object as BulletWorldObject
from ..designator import DesignatorDescription
from ..orm.base import (Position as ORMPosition, Quaternion as ORMQuaternion)
from ..orm.object_designator import (ObjectDesignator as ORMObjectDesignator, BelieveObject as ORMBelieveObject,
                                     ObjectPart as ORMObjectPart)


class ObjectDesignatorDescription(DesignatorDescription):
    """
    Class for object designator descriptions.
    Descriptions hold possible parameter ranges for object designators.
    """

    @dataclasses.dataclass
    class Object:
        """
        A single element that fits the description.
        """

        name: str
        """
        Name of the object
        """

        type: str
        """
        Type of the object
        """

        bullet_world_object: Optional[BulletWorldObject]
        """
        Reference to the BulletWorld object
        """

        _pose: Optional[Callable] = dataclasses.field(init=False)
        """
        A callable returning the pose of this object. The _pose member is used overwritten for data copies
        which will not update when the original bullet_world_object is moved.
        """

        def __post_init__(self):
            if self.bullet_world_object:
                self._pose = self.bullet_world_object.get_position_and_orientation

        def to_sql(self) -> ORMObjectDesignator:
            """
            Create an ORM object that corresponds to this description.

            :return: The created ORM object.
            """
            return ORMObjectDesignator(self.type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectDesignator:
            """
            Add and commit this and all related objects to the session.
            Auto-Incrementing primary keys and foreign keys have to be filled by this method.

            :param session: Session with a database that is used to add and commit the objects
            :return: The completely instanced ORM object
            """

            # insert position and orientation of object of the designator
            orm_position = ORMPosition(*self.pose[0])
            orm_orientation = ORMQuaternion(*self.pose[1])
            session.add(orm_position)
            session.add(orm_orientation)
            session.commit()

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

            # create object orm designator
            obj = self.to_sql()
            obj.position = orm_position.id
            obj.orientation = orm_orientation.id
            session.add(obj)
            session.commit()
            return obj

        def data_copy(self) -> 'ObjectDesignatorDescription.Object':
            """
            :return: A copy containing only the fields of this class. The BulletWorldObject attached to this pycram
            object is not copied. The _pose gets set to a method that statically returns the pose of the object when
            this method was called.
            """
            result = ObjectDesignatorDescription.Object(self.name, self.type, None)
            # get current object pose and set resulting pose to always be that
            pose = self.pose
            result.pose = lambda: pose
            return result

        @property
        def pose(self):
            """
            Property of the current position and orientation of the object.
            Evaluate the _pose function.

            :return: Position and orientation
            """
            return self._pose()

        @pose.setter
        def pose(self, value: Callable):
            """
            Set the pose to a new method that returns the current pose.

            :param value: A callable that returns a pose.
            """
            self._pose = value

        def __repr__(self):
            return self.__class__.__qualname__ + f"(" + ', '.join(
                [f"{f.name}={self.__getattribute__(f.name)}" for f in dataclasses.fields(self)] + [
                    f"pose={self.pose}"]) + ')'

    def __init__(self, names: Optional[List[str]] = None, types: Optional[List[str]] = None,
                 resolver: Optional[Callable] = None):
        """
        Base of all object designator descriptions. Every object designator has the name and type of the object.

        :param names: A list of names that could describe the object
        :param types: A list of types that could represent the object
        :param resolver: An alternative resolver that returns an object designator for the list of names and types
        """
        super().__init__(resolver)
        self.types: Optional[List[str]] = types
        self.names: Optional[List[str]] = names

    def ground(self) -> Union[Object, bool]:
        """
        Return the first object from the bullet world that fits the description.

        :return: A resolved object designator
        """
        return next(iter(self))

    def __iter__(self) -> Iterable[Object]:
        """
        Iterate through all possible objects fitting this description

        :yield: A resolved object designator
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
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):

        # The rest of attributes is inherited
        part_pose: Tuple[List[float], List[float]]

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
                yield self.Object(name, self.type, self.part_of,
                                  self.part_of.bullet_world_object.get_link_position_and_orientation(name))


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
