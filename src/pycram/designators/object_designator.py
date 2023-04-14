from typing import List, Tuple, Union

import sqlalchemy.orm

from ..bullet_world import Object
from ..designator import Designator, DesignatorError, DesignatorDescription
from ..orm.base import (Position as ORMPosition, Quaternion as ORMQuaternion)
from ..orm.object_designator import (ObjectDesignator as ORMObjectDesignator,
                                     BelieveObject as ORMBelieveObject,
                                     ObjectPart as ORMObjectPart)


class ObjectDesignator(Designator):
    resolvers = {}

    def __init__(self, description, parent=None):
        super().__init__(description, parent)

    def next_solution(self):
        try:
            self.reference()
        except DesignatorError:
            pass

        if self._solutions.has(self._index + 1):
            desig = ObjectDesignator(self._description, self)
            desig._solutions = self._solutions
            desig._index = self._index + 1
            return desig

        return None

    def ground(self):
        return None

    def get_slots(self):
        return self._description.get_slots()

    def __str__(self):
        return "ObjectDesignator({})".format(self._description.__dict__)


class ObjectDesignatorDescription(DesignatorDescription):
    type: str
    name: str
    object: Object
    pose: Tuple[List[float], List[float]]

    def __init__(self, type: str = None, name: str = None, object: Object = None, resolver="grounding"):
        super().__init__(resolver)
        self.type = type
        self.name = name
        self.object = object
        self.pose = None

    def to_sql(self) -> ORMObjectDesignator:
        return ORMObjectDesignator(self.type, self.name)

    def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectDesignator:
        self_ = self.to_sql()
        if self.pose:
            position = ORMPosition(*self.pose[0])
            orientation = ORMQuaternion(*self.pose[1])
            session.add(position)
            session.add(orientation)
            session.commit()
            self_.position = position.id
            self_.orientation = orientation.id
        else:
            self_.position = None
            self_.orientation = None

        session.add(self_)
        session.commit()
        return self_


class BelieveObject(ObjectDesignatorDescription):

    def __init__(self, type, name, object, resolver="grounding"):
        super().__init__(type, name, object, resolver)

    def to_sql(self) -> ORMBelieveObject:
        return ORMBelieveObject(self.type, self.name)

    def insert(self, session: sqlalchemy.orm.session.Session) -> ORMBelieveObject:
        self_ = self.to_sql()
        session.add(self_)
        session.commit()
        return self_


class ObjectPart(ObjectDesignatorDescription):
    part_of: Union[Object, ObjectDesignator]

    def __init__(self, type: str = None, name: str = None, object: Object = None, part_of=None,
                 resolver="grounding"):
        super(ObjectPart, self).__init__(type, name, object, resolver)
        self.part_of = part_of

    def to_sql(self) -> ORMObjectPart:
        return ORMObjectPart(self.type, self.name)

    def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectPart:
        self_ = self.to_sql()

        # try to create the part_of object
        if self.part_of:
            part = self.part_of.insert(session)
            self_.part_of = part.id
        else:
            self_.part_of = None

        session.add(self_)
        session.commit()

        return self_


class LocatedObject(ObjectDesignatorDescription):
    pose: List[float]
    reference_frame: str
    timestamp: float

    def __init__(self, type: str = None, name: str = None, pose: List[float] = None, reference_frame: str = "world",
                 timestamp: float = None, resolver="grounding"):
        super().__init__(type, name, resolver)
        self.pose = pose
        self.reference_frame = reference_frame
        self.timestamp = timestamp
