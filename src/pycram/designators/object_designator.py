from ..designator import Designator, DesignatorError, DesignatorDescription
from ..bullet_world import Object
from typing import List, Tuple, Union


class ObjectDesignator(Designator):
    """
    Object Designators are a symbolic representation of objects in the environment. This symbolic representation can
    be resolved to an actual object in the environment that fits all criteria given in the Object Designator description.
    There can be more than one possible solution to the criteria, in this case the first found object is returned and
    the rest are returned via the next_solution method.
    """

    resolvers = {}

    def __init__(self, description, parent=None):
        """
        Object Designators get a description which describes which kind of object is described and the parameter which
        describe this object.

        :param description: The description which describes the object of this designator.
        :param parent: The parent designator, designators are resolved in a chain where the next_solution is always the
        child designator of the previous solution.
        """
        super().__init__(description, parent)

    def next_solution(self):
        """
        Returns an object designator with the same type of description as this one. However, the solution to the returned
        designator is the next possible solution given the parameter in the description. If there is no possible next
        solution this method returns None.

        :return: A Object Designator which has the next possible solution or None if there is no next solution.
        """
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
        """
        Returns the name of parameters of the designator description of this designator.

        :return: A list with names of the description of this designator.
        """
        return self._description.get_slots()

    def __str__(self):
        return "ObjectDesignator({})".format(self._description.__dict__)


class ObjectDesignatorDescription(DesignatorDescription):
    """
    Parent class of all descriptions for an Object Designator,

    :ivar type: Type of the object
    :ivar name: The name of the object
    :ivar object: The BulletWorld Object reference
    :ivar pose: The position and orientation of the object
    """
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


class BelieveObject(ObjectDesignatorDescription):
    """
    A wrapper around the ObjectDesignatorDescripton which describes an object in the believe state (BulletWorld). The
    parameter for this description are the same as for ObjectDesignatorDescription.
    """

    def __init__(self, type=None, name=None, object=None, resolver="grounding"):
        super().__init__(type, name, object, resolver)


class ObjectPart(ObjectDesignatorDescription):
    """
    Describes a part of an object, for example, a drawer as part of the kitchen.

    :ivar type: Which type the part is, for example, a drawer
    :ivar name: The name of the URDF link that represents the part
    :ivar part_of: The BulletWorld Object reference to which the part described is a part of
    """
    type: str
    name: str
    part_of: Union[Object, ObjectDesignator]

    def __init__(self, type: str, name: str, part_of: Object = None, resolver="grounding"):
        super().__init__(type, name, resolver)
        #self.type = type
        #self.name = name
        self.part_of = part_of


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
