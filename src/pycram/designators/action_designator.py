import itertools
from typing_extensions import List, Union, Callable
from .object_designator import ObjectDesignatorDescription, ObjectPart
from ..enums import Arms
from ..designator import ActionDesignatorDescription
from ..pose import Pose
from pycram.designators.actions.actions import (ParkArmsActionPerformable, MoveTorsoActionPerformable,
                                                SetGripperActionPerformable, GripActionPerformable,
                                                PlaceActionPerformable, PickUpActionPerformable,
                                                NavigateActionPerformable, TransportActionPerformable,
                                                LookAtActionPerformable, DetectActionPerformable, OpenActionPerformable,
                                                CloseActionPerformable, GraspingActionPerformable,
                                                ReleaseActionPerformable)


class MoveTorsoAction(ActionDesignatorDescription):
    """
    Action Designator for Moving the torso of the robot up and down
    """

    def __init__(self, positions: List[float], resolver=None):
        """
        Create a designator description to move the torso of the robot up and down.

        :param positions: List of possible positions of the robots torso, possible position is a float of height in metres
        :param resolver: An optional resolver that returns a performable designator for a designator description.
        """
        super().__init__(resolver)
        self.positions: List[float] = positions

    def ground(self) -> MoveTorsoActionPerformable:
        """
        Creates a performable action designator with the first element from the list of possible torso heights.

        :return: A performable action designator
        """
        return MoveTorsoActionPerformable(self.positions[0])

    def __iter__(self):
        """
        Iterates over all possible values for this designator and returns a performable action designator with the value.

        :return: A performable action designator
        """
        for position in self.positions:
            yield MoveTorsoActionPerformable(position)


class SetGripperAction(ActionDesignatorDescription):
    """
    Set the gripper state of the robot
    """

    def __init__(self, grippers: List[str], motions: List[str], resolver=None):
        """
        Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

        :param grippers: A list of possible grippers
        :param motions: A list of possible motions
        :param resolver: An alternative resolver that returns a performable designator for a designator description
        """
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.motions: List[str] = motions

    def ground(self) -> SetGripperActionPerformable:
        """
        Default resolver that returns a performable designator with the first element in the grippers and motions list.

        :return: A performable designator
        """
        return SetGripperActionPerformable(self.grippers[0], self.motions[0])

    def __iter__(self):
        """
        Iterates over all possible combinations of grippers and motions

        :return: A performable designator with a combination of gripper and motion
        """
        for parameter_combination in itertools.product(self.grippers, self.motions):
            yield SetGripperActionPerformable(*parameter_combination)


class ReleaseAction(ActionDesignatorDescription):
    """
    Releases an Object from the robot.

    Note: This action can not be used yet.
    """

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription,
                 resolver=None):
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description = object_designator_description

    def ground(self) -> ReleaseActionPerformable:
        return ReleaseActionPerformable(self.grippers[0], self.object_designator_description.ground())


class GripAction(ActionDesignatorDescription):
    """
    Grip an object with the robot.

    :ivar grippers: The grippers to consider
    :ivar object_designator_description: The description of objects to consider
    :ivar efforts: The efforts to consider

    Note: This action can not be used yet.
    """

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription,
                 efforts: List[float], resolver=None):
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.efforts: List[float] = efforts

    def ground(self) -> GripActionPerformable:
        return GripActionPerformable(self.grippers[0], self.object_designator_description.ground(), self.efforts[0])


class ParkArmsAction(ActionDesignatorDescription):
    """
    Park the arms of the robot.
    """

    def __init__(self, arms: List[Arms], resolver=None):
        """
        Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

        :param arms: A list of possible arms, that could be used
        :param resolver: An optional resolver that returns a performable designator from the designator description
        """
        super().__init__(resolver)
        self.arms: List[Arms] = arms

    def ground(self) -> ParkArmsActionPerformable:
        """
        Default resolver that returns a performable designator with the first element of the list of possible arms

        :return: A performable designator
        """
        return ParkArmsActionPerformable(self.arms[0])


class PickUpAction(ActionDesignatorDescription):
    """
    Designator to let the robot pick up an object.
    """

    def __init__(self, object_designator_description:  Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[str], grasps: List[str], resolver=None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def ground(self) -> PickUpActionPerformable:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return PickUpActionPerformable(obj_desig, self.arms[0], self.grasps[0])


class PlaceAction(ActionDesignatorDescription):
    """
    Places an Object at a position using an arm.
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 target_locations: List[Pose],
                 arms: List[str], resolver=None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        :param resolver: Grounding method to resolve this designator
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.target_locations: List[Pose] = target_locations
        self.arms: List[str] = arms

    def ground(self) -> PlaceActionPerformable:
        """
        Default resolver that returns a performable designator with the first entries from the list of possible entries.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return PlaceActionPerformable(obj_desig, self.arms[0], self.target_locations[0])


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    def __init__(self, target_locations: List[Pose], resolver=None):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param resolver: An alternative resolver that creates a performable designator from the list of possible parameter
        """
        super().__init__(resolver)
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> NavigateActionPerformable:
        """
        Default resolver that returns a performable designator with the first entry of possible target locations.

        :return: A performable designator
        """
        return NavigateActionPerformable(self.target_locations[0])


class TransportAction(ActionDesignatorDescription):
    """
    Transports an object to a position using an arm
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[str],
                 target_locations: List[Pose], resolver=None):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator description or a specified Object designator that should be transported
        :param arms: A List of possible arms that could be used for transporting
        :param target_locations: A list of possible target locations for the object to be placed
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> TransportActionPerformable:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description \
            if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object)\
            else self.object_designator_description.resolve()

        return TransportActionPerformable(obj_desig, self.arms[0], self.target_locations[0])


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    def __init__(self, targets: List[Pose], resolver=None):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        :param resolver: An alternative resolver that returns a performable designator for a list of possible target locations
        """
        super().__init__(resolver)
        self.targets: List[Pose] = targets

    def ground(self) -> LookAtActionPerformable:
        """
        Default resolver that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return LookAtActionPerformable(self.targets[0])


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object that fits the object description and returns an object designator describing the object.
    """

    def __init__(self, object_designator_description: ObjectDesignatorDescription, resolver=None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param object_designator_description: Object designator describing the object
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description

    def ground(self) -> DetectActionPerformable:
        """
        Default resolver that returns a performable designator with the resolved object description.

        :return: A performable designator
        """
        return DetectActionPerformable(self.object_designator_description.resolve())


class OpenAction(ActionDesignatorDescription):
    """
    Opens a container like object

    Can currently not be used
    """

    def __init__(self, object_designator_description: ObjectPart, arms: List[str], resolver=None):
        """
        Moves the arm of the robot to open a container.

        :param object_designator_description: Object designator describing the handle that should be used to open
        :param arms: A list of possible arms that should be used
        :param resolver: A alternative resolver that returns a performable designator for the lists of possible parameter.
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> OpenActionPerformable:
        """
        Default resolver that returns a performable designator with the resolved object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator
        """
        return OpenActionPerformable(self.object_designator_description.resolve(), self.arms[0])


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container like object.

    Can currently not be used
    """

    def __init__(self, object_designator_description: ObjectPart, arms: List[str],
                 resolver=None):
        """
        Attempts to close an open container

        :param object_designator_description: Object designator description of the handle that should be used
        :param arms: A list of possible arms to use
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> CloseActionPerformable:
        """
        Default resolver that returns a performable designator with the resolved object designator and the first entry from
        the list of possible arms.

        :return: A performable designator
        """
        return CloseActionPerformable(self.object_designator_description.resolve(), self.arms[0])


class GraspingAction(ActionDesignatorDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    def __init__(self, arms: List[str], object_description: Union[ObjectDesignatorDescription, ObjectPart],
                 resolver: Callable = None):
        """
        Will try to grasp the object described by the given description. Grasping is done by moving into a pre grasp
        position 10 cm before the object, opening the gripper, moving to the object and then closing the gripper.

        :param arms: List of Arms that should be used for grasping
        :param object_description: Description of the object that should be grasped
        :param resolver: An alternative resolver to get a specified designator from the designator description
        """
        super().__init__(resolver)
        self.arms: List[str] = arms
        self.object_description: ObjectDesignatorDescription = object_description

    def ground(self) -> GraspingActionPerformable:
        """
        Default resolver that takes the first element from the list of arms and the first solution for the object
        designator description ond returns it.

        :return: A performable action designator that contains specific arguments
        """
        return GraspingActionPerformable(self.arms[0], self.object_description.resolve())
