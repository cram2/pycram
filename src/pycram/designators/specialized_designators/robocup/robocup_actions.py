from typing import List, Optional, Type, Any

import rospy
from owlready2 import Thing

from ...action_designator import ActionAbstract, PouringActionPerformable
from ...location_designator import CostmapLocation
from ...motion_designator import MoveJointsMotion, MoveGripperMotion, MoveArmJointsMotion, MoveTCPMotion, MoveMotion, \
    LookingMotion, DetectingMotion, OpeningMotion, ClosingMotion, HalfOpeningDishwasherMotion, MoveArmAroundMotion, \
    GraspingDishwasherHandleMotion, FullOpeningDishwasherMotion
from ...object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ....local_transformer import LocalTransformer
from ....plan_failures import ObjectUnfetchable, ReachabilityFailure
# from ..robot_descriptions import robot_description
from ....robot_description import RobotDescription
from ....robot_descriptions import robot_description
from ....tasktree import with_tree

from owlready2 import Thing

from ....datastructures.enums import Arms, Grasp, GripperState
from ....designator import ActionDesignatorDescription
from ....datastructures.pose import Pose
from ....datastructures.world import World
from ....ontology.ontology import OntologyConceptHolder

from ....orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                       PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                       MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                       LookAtAction as ORMLookAtAction, DetectAction as ORMDetectAction,
                                       TransportAction as ORMTransportAction, OpenAction as ORMOpenAction,
                                       CloseAction as ORMCloseAction, GraspingAction as ORMGraspingAction, Action,
                                       FaceAtAction as ORMFaceAtAction, ParkArmsAction)
from ....orm.base import Pose as ORMPose
from ....orm.object_designator import Object as ORMObject
from ....orm.action_designator import Action as ORMAction
from dataclasses import dataclass, field

from ....utils import axis_angle_to_quaternion
from ....worlds.bullet_world import BulletWorld


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    def __init__(self, targets: List[Pose], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        :param resolver: An alternative specialized_designators that returns a performable designator for a list of possible target locations
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.targets: List[Pose] = targets

        if self.soma:
            self.init_ontology_concepts({"looking_at": self.soma.LookingAt})

    def ground(self) -> LookAtActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return LookAtActionPerformable(self.targets[0])


class PouringAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a pouring action.
    """

    def __init__(self, target_locations: List[Pose], arms: List[Arms], directions: List[str], angles: List[float],
                 resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        :param target_locations: List of possible target locations to be poured into
        :param arms: List of possible arms that could be used
        :param directions: List of possible directions for the pouring direction
        :param angles: List of possible angles that the gripper tilts to
        :param resolver: An alternative specialized_designators that returns a performable designator
                         for a list of possible target locations
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as
                                         or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.target_locations: List[Pose] = target_locations
        self.arms: List[Arms] = arms
        self.directions: List[str] = directions
        self.angels: List[float] = angles

        if self.soma:
            self.init_ontology_concepts({"pouring": self.soma.Pouring})

    def ground(self) -> PouringActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list
        of possible targets

        :return: A performable designator
        """
        return PouringActionPerformable(self.targets[0])


class MixingAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a mixing action.
    """

    def __init__(self, object_designator_description: ObjectDesignatorDescription,
                 object_tool_designator_description: ObjectDesignatorDescription, arms: List[Arms], grasps: List[str],
                 object_at_execution: Optional[ObjectDesignatorDescription.Object], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Initialize the MixingAction with object and tool designators, arms, and grasps.

        :param object_designator_description: Object designator for the object to be mixed.
        :param object_tool_designator_description: Object designator for the mixing tool.
        :param arms: List of possible arms that could be used.
        :param grasps: List of possible grasps for the mixing action.
        :param object_at_execution: The object at the time this Action got created. It is used to be a static, information holding entity.
                                    It is not updated when the BulletWorld object is changed.
        :param resolver: An optional resolver for dynamic parameter selection.
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super(MixingAction, self).__init__(resolver, ontology_concept_holders)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.object_tool_designator_description: ObjectDesignatorDescription = object_tool_designator_description
        self.arms: List[Arms] = arms
        self.grasps: List[str] = grasps

        if self.soma:
            self.init_ontology_concepts({"mixing": self.soma.Mixing})

    def ground(self) -> MixingActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return MixingActionPerformable(self.targets[0])


class HeadFollowAction(ActionDesignatorDescription):
    """
    Continuously move head to human closest to robot
    """

    def __init__(self, state: str, resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        :param state: defines if the robot should start/stop looking at human
        :param resolver: An optional resolver that returns a performable designator from the designator description
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.state = state

        if self.soma:
            self.init_ontology_concepts({"headfollow": self.soma.Headfollow})

    def ground(self) -> HeadFollowActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry
        in the list of possible targets

        :return: A performable designator
        """
        return HeadFollowActionPerformable(self.targets[0])


class DoorOpenAction(ActionDesignatorDescription):
    """
    grasp and open door
    """

    def __init__(self, handle: str, resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        :param handle: handle in tf to grasp
        :param resolver: An optional resolver that returns a performable designator from the designator description
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.handle = handle

        if self.soma:
            self.init_ontology_concepts({"door_open": self.soma.DoorOpen})

    def ground(self) -> DoorOpenActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return DoorOpenActionPerformable(self.targets[0])


class CuttingAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a cutting action.
    """

    def __init__(self, object_designator_description: ObjectDesignatorDescription, arms: List[Arms], grasps: List[str],
                 resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Initialize the CuttingAction with object designators, arms, and grasps.

        :param object_designator_description: Object designator for the object to be cut.
        :param arms: List of possible arms that could be used.
        :param grasps: List of possible grasps for the cutting action.
        :param resolver: An optional resolver for dynamic parameter selection.
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super(CuttingAction, self).__init__(resolver, ontology_concept_holders)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.arms: List[Arms] = arms
        self.grasps: List[str] = grasps

        if self.soma:
            self.init_ontology_concepts({"cutting": self.soma.Cutting})

    # TODO: just delete?
    # def __iter__(self):
    #     for object_, grasp, arm in itertools.product(iter(self.object_designator_description), self.grasps, self.arms):
    #         yield self.Action(object_, arm, grasp, slice_thickness=0.05, tool="big_knife", technique="slicing")

    def ground(self) -> CuttingActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return CuttingActionPerformable(self.targets[0])


class OpenDishwasherAction(ActionDesignatorDescription):
    """
    Opens the dishwasher door
    """

    def __init__(self, handle_name: str, door_name: str, goal_state_half_open: float, goal_state_full_open: float,
                 arms: List[Arms], resolver=None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Moves the arm of the robot to open a container.

        :param handle_name: name of the dishwasher handle
        :param door_name: name of the belonging dishwasher door
        :param goal_state_half_open: state to open the dishwasher door partially
        :param goal_state_full_open: state to open the dishwasher door fully
        :param arms: A list of possible arms that should be used
        :param resolver: A alternative resolver that returns a performable designator for the lists of possible parameter
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.handle_name = handle_name
        self.door_name = door_name
        self.goal_state_half_open = goal_state_half_open
        self.goal_state_full_open = goal_state_full_open
        self.arms: List[Arms] = arms

        if self.soma:
            self.init_ontology_concepts({"open_dishwasher": self.soma.OpenDishwasher})

    def ground(self) -> OpenDishwasherActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return OpenDishwasherActionPerformable(self.targets[0])


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object with given technique.
    """

    def __init__(self, technique, resolver=None,
                 object_designator: Optional[ObjectDesignatorDescription] = None,
                 state: Optional[str] = None, ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param technique: Technique means how the object should be detected, e.g. 'color', 'shape', etc.
        :param object_designator_description: Object designator describing the object
        :param state: The state instructs our perception system to either start or stop the search for an object or human.
        :param resolver: An alternative resolver
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.technique: str = technique
        self.object_designator: Optional[ObjectDesignatorDescription] = object_designator
        self.state: Optional[str] = state

        if self.soma:
            self.init_ontology_concepts({"detect": self.soma.Detect})

    def ground(self) -> DetectActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return DetectActionPerformable(self.targets[0])


class PlaceGivenObjAction(ActionDesignatorDescription):
    """
    A class representing a designator for a place action of human given objects, allowing a robot to place a
    human given object, that could not be picked up or were not found in the FOV.

    This class encapsulates the details of the place action of human given objects, including the type of the object to
    be placed, the arm to be used, the target_location to place the object and the grasp type. It defines the sequence
    of operations for the robot to execute the place action of human given object, such as moving the arm holding the
    object to the target_location, opening the gripper, and lifting the arm.
    """

    def __init__(self,
                 object_types: List[str], arms: List[Arms], target_locations: List[Pose], grasps: List[str],
                 on_table: Optional[bool] = True, resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Lets the robot place a human given object. The description needs an object type describing the object that
        should be placed, an arm that should be used as well as the target location where the object should be placed
        and the needed grasping movement.

        :param object_types: List of possible object types
        :param arms: List of possible arms that could be used
        :param target_locations: List of possible target locations for the object to be placed
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of
                         possible paramter
        :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(resolver, ontology_concept_holders)
        self.object_types: List[str] = object_types
        self.arms: List[Arms] = arms
        self.grasps: List[str] = grasps
        self.target_locations: List[Pose] = target_locations
        self.on_table: bool = on_table

        if self.soma:
            self.init_ontology_concepts({"place_given_obj": self.soma.PlaceGivenObj})

    def ground(self) -> PlaceGivenObjActionPerformable:
        """
        Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return PlaceGivenObjActionPerformable(self.targets[0])


# ----------------------------------------------------------------------------
# ---------------- Performables ----------------------------------------------
# ----------------------------------------------------------------------------


@dataclass
class ActionAbstract(ActionDesignatorDescription.Action, abc.ABC):
    """Base class for performable performables."""
    orm_class: Type[ORMAction] = field(init=False, default=None)
    """
    The ORM class that is used to insert this action into the database. Must be overwritten by every action in order to
    be able to insert the action into the database.
    """

    @abc.abstractmethod
    def perform(self) -> None:
        """
        Perform the action.

        Will be overwritten by each action.
        """
        pass

    def to_sql(self) -> Action:
        """
        Convert this action to its ORM equivalent.

        Needs to be overwritten by an action if it didn't overwrite the orm_class attribute with its ORM equivalent.

        :return: An instance of the ORM equivalent of the action with the parameters set
        """
        # get all class parameters (ignore inherited ones)
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters (ignore inherited ones)
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # list of parameters that will be passed to the ORM class. If the name does not match the orm_class equivalent
        # or if it is a type that needs to be inserted into the session manually, it will not be added to the list
        parameters = [value for key, value in class_variables.items() if key in orm_class_variables
                      and not isinstance(value, (ObjectDesignatorDescription.Object, Pose))]

        return self.orm_class(*parameters)

    def insert(self, session: Session, **kwargs) -> Action:
        """
        Insert this action into the database.

        Needs to be overwritten by an action if the action has attributes that do not exist in the orm class
        equivalent. In that case, the attributes need to be inserted into the session manually.

        :param session: Session with a database that is used to add and commit the objects
        :param kwargs: Possible extra keyword arguments
        :return: The completely instanced ORM action that was inserted into the database
        """

        action = super().insert(session)

        # get all class parameters (ignore inherited ones)
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters (ignore inherited ones)
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # loop through all class parameters and insert them into the session unless they are already added by the ORM
        for key, value in class_variables.items():
            if key not in orm_class_variables:
                variable = value.insert(session)
                if isinstance(variable, ORMObject):
                    action.object = variable
                elif isinstance(variable, ORMPose):
                    action.pose = variable
        session.add(action)

        return action


@dataclass
class PlaceGivenObjActionPerformable(ActionAbstract):
    """
    place action of human given objects, allowing a robot to place a
    human given object, that could not be picked up or were not found in the FOV.
    """

    object_type: str
    """
    Object type describing the object that should be placed
    """

    arm: str
    """
    Arm that is currently holding the object
    """

    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """

    grasp: str
    """
    Grasp that defines how to place the given object
    """

    on_table: Optional[bool]
    """
    When placing a plate needed to differentiate between placing in a dishwasher and placing on the table. 
    Default is placing on a table.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> None:
        lt = LocalTransformer()
        robot = BulletWorld.robot
        # oTm = Object Pose in Frame map
        oTm = self.target_location
        execute = True

        # TODO add for other robots
        if self.object_type == "Metalplate" and self.on_table and robot.name == "hsrb":

            grasp_rotation = robot_description.grasps.get_orientation_for_grasp("front")
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            oTb.orientation = grasp_rotation
            oTmG = lt.transform_pose(oTb, "map")

            rospy.logwarn("Placing now")
            MoveTCPMotion(oTmG, self.arm).resolve().perform()

            MoveTorsoAction([0.62]).resolve().perform()
            kwargs = dict()

            # taking in the predefined arm configuration for placing
            if self.arm in ["left", "both"]:
                kwargs["left_arm_config"] = "place_plate"
                MoveArmJointsMotion(**kwargs).resolve().perform()

            # turning the gripper downwards to better drop the plate
            MoveJointsMotion(["wrist_flex_joint"], [-0.8]).resolve().perform()

            # correct a possible sloped orientation
            NavigateAction(
                [Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y,
                       0])]).resolve().perform()

            MoveGripperMotion(motion="open", gripper="left").resolve().perform()

            # Move away from the table
            # todo generalize so that hsr is always moving backwards
            NavigateAction(
                [Pose([robot.get_pose().pose.position.x - 0.1, robot.get_pose().pose.position.y,
                       0])]).resolve().perform()

        # placing everything else or the Metalplate in the dishwasher
        else:
            if self.grasp == "top":
                oTm.pose.position.z += 0.05

            # Determine the grasp orientation and transform the pose to the base link frame
            grasp_rotation = robot_description.grasps.get_orientation_for_grasp(self.grasp)
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            # Set pose to the grasp rotation
            oTb.orientation = grasp_rotation
            # Transform the pose to the map frame
            oTmG = lt.transform_pose(oTb, "map")

            rospy.logwarn("Placing now")
            BulletWorld.current_bullet_world.add_vis_axis(oTmG)
            if execute:
                MoveTCPMotion(oTmG, self.arm).resolve().perform()

            tool_frame = robot_description.get_tool_frame(self.arm)
            push_base = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
            if robot.name == "hsrb":
                z = 0.03
                if self.grasp == "top":
                    z = 0.07
                push_base.pose.position.z += z
            # todo: make this for other robots
            push_baseTm = lt.transform_pose(push_base, "map")

            rospy.logwarn("Pushing now")
            BulletWorld.current_bullet_world.add_vis_axis(push_baseTm)
            if execute:
                MoveTCPMotion(push_baseTm, self.arm).resolve().perform()
            if self.object_type == "Metalplate":
                loweringTm = push_baseTm
                loweringTm.pose.position.z -= 0.08
                BulletWorld.current_bullet_world.add_vis_axis(loweringTm)
                if execute:
                    MoveTCPMotion(loweringTm, self.arm).resolve().perform()
                # rTb = Pose([0,-0.1,0], [0,0,0,1],"base_link")
                rospy.logwarn("sidepush monitoring")
                TalkingMotion("sidepush.").resolve().perform()
                side_push = Pose(
                    [push_baseTm.pose.position.x, push_baseTm.pose.position.y + 0.125, loweringTm.pose.position.z],
                    [push_baseTm.orientation.x, push_baseTm.orientation.y, push_baseTm.orientation.z,
                     push_baseTm.orientation.w])
                try:
                    plan = MoveTCPMotion(side_push, self.arm) >> Monitor(monitor_func)
                    plan.perform()
                except (SensorMonitoringCondition):
                    rospy.logwarn("Open Gripper")
                    MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            # Finalize the placing by opening the gripper and lifting the arm
            rospy.logwarn("Open Gripper")
            MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            rospy.logwarn("Lifting now")
            liftingTm = push_baseTm
            liftingTm.pose.position.z += 0.08
            BulletWorld.current_bullet_world.add_vis_axis(liftingTm)
            if execute:
                MoveTCPMotion(liftingTm, self.arm).resolve().perform()


@dataclass
class DetectActionPerformable(ActionAbstract):
    """
    Detects an object with given technique.
    """

    technique: str
    """
    Technique means how the object should be detected, e.g. 'color', 'shape', 'region', etc. 
    Or 'all' if all objects should be detected
    """
    object_designator: Optional[ObjectDesignatorDescription] = None
    """
    Object designator loosely describing the object, e.g. only type. 
    """
    state: Optional[str] = None
    """
    The state instructs our perception system to either start or stop the search for an object or human.
    Can also be used to describe the region or location where objects are perceived.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> Any:
        if self.object_designator:
            object_type = self.object_designator.types[0]
        else:
            object_type = None
        return DetectingMotion(technique=self.technique, object_type=object_type,
                               state=self.state).resolve().perform()


@dataclass
class OpenDishwasherActionPerformable(ActionAbstract):
    """
    Opens the dishwasher door
     """

    handle_name: str
    """
    Name of the handle to grasp for opening
    """
    door_name: str
    """
    Name of the door belonging to the handle
    """
    goal_state_half_open: float
    """
    goal state for opening the door partially
    """
    goal_state_full_open: float
    """
    goal state for opening the door fully
    """
    arm: str
    """
    Arm that should be used for opening the container
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> Any:
        # Grasping the dishwasher handle
        MoveGripperMotion("open", self.arm).resolve().perform()
        GraspingDishwasherHandleMotion(self.handle_name, self.arm).resolve().perform()

        # partially opening the dishwasher door
        MoveGripperMotion("close", self.arm).resolve().perform()
        HalfOpeningDishwasherMotion(self.handle_name, self.goal_state_half_open, self.arm).resolve().perform()

        # moves arm around the door to further push it open
        MoveGripperMotion("open", self.arm).resolve().perform()
        MoveArmAroundMotion(self.handle_name, self.arm).resolve().perform()

        # pushes the rest of the door open
        MoveGripperMotion("close", self.arm).resolve().perform()
        FullOpeningDishwasherMotion(self.handle_name, self.door_name, self.goal_state_full_open,
                                    self.arm).resolve().perform()

        ParkArmsAction([self.arm]).resolve().perform()
        MoveGripperMotion("open", self.arm).resolve().perform()
        # plan = talk | park | gripper_open
        # plan.perform()


@dataclass
class CuttingActionPerformable(ActionAbstract):
    """
    Designator to let the robot perform a cutting action.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be cut.
    """

    arm: str
    """
    The arm that should be used for cutting.
    """

    grasp: str
    """
    The grasp that should be used for cutting. For example, 'left' or 'right'.
    """

    slice_thickness: float
    """
    The upper bound thickness of the slices.
    """

    tool: str
    """
    The tool to cut with.
    """

    technique: str
    """
    Technique used to cut the object.
    """

    # TODO: needed parameter?
    object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False, repr=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> None:
        """
        Perform the cutting action using the specified object, arm, grasp, slice thickness, tool, and technique.
        """
        # Store the object's data copy at execution
        self.object_at_execution = self.object_designator.data_copy()

        # Get grasp orientation and target pose
        grasp = robot_description.grasps.get_orientation_for_grasp(self.grasp)
        # Retrieve object and robot from designators
        object = self.object_designator.bullet_world_object
        obj_dim = object.get_object_dimensions()

        dim = [max(obj_dim[0], obj_dim[1]), min(obj_dim[0], obj_dim[1]), obj_dim[2]]
        oTm = object.get_pose()
        object_pose = object.local_transformer.transform_to_object_frame(oTm, object)

        # from bread_dim calculate def a calculation that gets me the highest number from the first 2 entries
        # Given slice thickness is 3 cm or 0.03 meters
        slice_thickness = self.slice_thickness
        # Calculate slices and transform them to the map frame with orientation
        obj_length = dim[0]
        obj_width = dim[1]
        obj_height = dim[2]

        # Calculate the starting Y-coordinate offset (half the width minus half a slice thickness)
        if self.technique == 'halving':
            start_offset = 0
            num_slices = 1
        else:
            num_slices = 1
            # int(obj_length // slice_thickness))
            start_offset = 0  # -obj_length / 2 + slice_thickness / 2)

        # Calculate slice coordinates
        slice_coordinates = [start_offset + i * slice_thickness for i in range(num_slices)]

        # Transform slice coordinates to map frame with orientation
        slice_poses = []
        for x in slice_coordinates:
            tmp_pose = object_pose.copy()
            tmp_pose.pose.position.y -= 3 * obj_width
            tmp_pose.pose.position.x = x
            sTm = object.local_transformer.transform_pose(tmp_pose, "map")
            slice_poses.append(sTm)

        for slice_pose in slice_poses:
            # rotate the slice_pose by grasp
            ori = multiply_quaternions(
                [slice_pose.orientation.x, slice_pose.orientation.y, slice_pose.orientation.z,
                 slice_pose.orientation.w], grasp)

            oriR = axis_angle_to_quaternion([0, 0, 1], 90)
            oriM = multiply_quaternions([oriR[0], oriR[1], oriR[2], oriR[3]], [ori[0], ori[1], ori[2], ori[3]])

            adjusted_slice_pose = slice_pose.copy()

            # Set the orientation of the object pose by grasp in MAP
            adjusted_slice_pose.orientation.x = oriM[0]
            adjusted_slice_pose.orientation.y = oriM[1]
            adjusted_slice_pose.orientation.z = oriM[2]
            adjusted_slice_pose.orientation.w = oriM[3]

            # Adjust the position of the object pose by grasp in MAP
            lift_pose = adjusted_slice_pose.copy()
            lift_pose.pose.position.z += 2 * obj_height
            # Perform the motion for lifting the tool
            BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
            MoveTCPMotion(lift_pose, self.arm).resolve().perform()
            # Perform the motion for cutting the object
            BulletWorld.current_bullet_world.add_vis_axis(adjusted_slice_pose)
            MoveTCPMotion(adjusted_slice_pose, self.arm).resolve().perform()
            # Perform the motion for lifting the tool
            BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
            MoveTCPMotion(lift_pose, self.arm).resolve().perform()


@dataclass
class DoorOpenActionPerformable(ActionAbstract):
    """
    grasp and open door
    """

    handle: str
    """
    defines the handle of the door to open
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> None:
        MoveGripperMotion(motion="open", gripper="left").resolve().perform()
        GraspHandleMotion(self.handle).resolve().perform()
        MoveGripperMotion(motion="close", gripper="left").resolve().perform()
        DoorOpenMotion(self.handle).resolve().perform()
        MoveGripperMotion(motion="open", gripper="left").resolve().perform()


@dataclass
class HeadFollowActionPerformable(ActionAbstract):
    """
    Continuously move head to human closest to robot
    """

    state: str
    """
    defines if the robot should start/stop looking at human
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> None:
        HeadFollowMotion(self.state).resolve().perform()


@dataclass
class MixingActionPerformable(ActionAbstract):
    """
    Lets the robot perform a mixing action.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the object that should be mixed.
    """

    object_tool_designator: ObjectDesignatorDescription.Object
    """
    Object designator describing the mixing tool.
    """

    arm: str
    """
    The arm that should be used for mixing.
    """

    grasp: str
    """
    The grasp that should be used for mixing. For example, 'left' or 'right'.
    """

    # TODO: still relevant?
    object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. 
    It is not updated when the BulletWorld object is changed.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> None:
        """
        Perform the mixing action using the specified object, tool, arm, and grasp.
        """
        # Store the object's data copy at execution
        self.object_at_execution = self.object_designator.data_copy()
        # Retrieve object and robot from designators
        object = self.object_designator.bullet_world_object

        obj_dim = object.get_object_dimensions()

        dim = [max(obj_dim[0], obj_dim[1]), min(obj_dim[0], obj_dim[1]), obj_dim[2]]
        obj_height = dim[2]
        oTm = object.get_pose()
        object_pose = object.local_transformer.transform_to_object_frame(oTm, object)

        def generate_spiral(pose, upward_increment, radial_increment, angle_increment, steps):
            x_start, y_start, z_start = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            spiral_poses = []

            for t in range(2 * steps):
                tmp_pose = pose.copy()

                r = radial_increment * t
                a = angle_increment * t
                h = upward_increment * t

                x = x_start + r * math.cos(a)
                y = y_start + r * math.sin(a)
                z = z_start + h

                tmp_pose.pose.position.x += x
                tmp_pose.pose.position.y += y
                tmp_pose.pose.position.z += z

                spiralTm = object.local_transformer.transform_pose(tmp_pose, "map")
                spiral_poses.append(spiralTm)
                BulletWorld.current_bullet_world.add_vis_axis(spiralTm)

            return spiral_poses

        # this is a very good one but takes ages
        # spiral_poses = generate_spiral(object_pose, 0.0004, 0.0008, math.radians(10), 100)
        spiral_poses = generate_spiral(object_pose, 0.001, 0.0035, math.radians(30), 10)

        BulletWorld.current_bullet_world.remove_vis_axis()
        for spiral_pose in spiral_poses:
            oriR = axis_angle_to_quaternion([1, 0, 0], 180)
            ori = multiply_quaternions(
                [spiral_pose.orientation.x, spiral_pose.orientation.y, spiral_pose.orientation.z,
                 spiral_pose.orientation.w], oriR)
            adjusted_slice_pose = spiral_pose.copy()
            # # Set the orientation of the object pose by grasp in MAP
            adjusted_slice_pose.orientation.x = ori[0]
            adjusted_slice_pose.orientation.y = ori[1]
            adjusted_slice_pose.orientation.z = ori[2]
            adjusted_slice_pose.orientation.w = ori[3]

            # Adjust the position of the object pose by grasp in MAP
            lift_pose = adjusted_slice_pose.copy()
            lift_pose.pose.position.z += (obj_height + 0.08)
            # Perform the motion for lifting the tool
            # BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
            MoveTCPMotion(lift_pose, self.arm).resolve().perform()


@dataclass
class PouringActionPerformable(ActionAbstract):
    """
    Lets the robot perform a pouring action
    """

    target_location: Pose
    """
    List of possible target locations to be poured into
    """
    arm: Arms
    """
    The arm to use
    """
    direction: str
    """
    pouring direction
    """
    angle: float
    """
    angle that the gripper tilts to
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPouringAction)

    @with_tree
    def perform(self) -> None:

        # Initialize the local transformer and robot reference
        lt = LocalTransformer()
        robot = BulletWorld.robot
        # Retrieve object and robot from designators

        # Calculate the object's pose in the map frame
        oTm = self.target_location
        execute = True
        # BulletWorld.current_bullet_world.add_vis_axis(oTm)

        # Determine the grasp orientation and transform the pose to the base link frame
        grasp_rotation = robot_description.grasps.get_orientation_for_grasp("front")
        oTbs = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
        oTbs.pose.position.x += 0.009  # was 0,009
        oTbs.pose.position.z += 0.17  # was 0.13

        if self.direction == "right":
            oTbs.pose.position.y -= 0.125
        else:
            oTbs.pose.position.y += 0.125

        oTms = lt.transform_pose(oTbs, "map")
        BulletWorld.current_bullet_world.add_vis_axis(oTms)

        oTog = lt.transform_pose(oTms, robot.get_link_tf_frame("base_link"))
        oTog.orientation = grasp_rotation
        oTgm = lt.transform_pose(oTog, "map")
        BulletWorld.current_bullet_world.add_vis_axis(oTgm)

        if self.direction == "right":
            new_q = axis_angle_to_quaternion([0, 0, 1], -self.angle)
        else:
            new_q = axis_angle_to_quaternion([0, 0, 1], self.angle)
        new_ori = multiply_quaternions(
            [oTgm.orientation.x, oTgm.orientation.y, oTgm.orientation.z,
             oTgm.orientation.w], new_q)
        oTmsp = oTgm.copy()
        oTmsp.pose.orientation.y = new_ori[1]
        oTmsp.pose.orientation.z = new_ori[2]
        oTmsp.pose.orientation.w = new_ori[3]
        BulletWorld.current_bullet_world.add_vis_axis(oTmsp)

        if execute:
            MoveTCPMotion(oTgm, self.arm, allow_gripper_collision=False).resolve().perform()
            MoveTCPMotion(oTmsp, self.arm, allow_gripper_collision=False).resolve().perform()
            MoveTCPMotion(oTgm, self.arm, allow_gripper_collision=False).resolve().perform()



@dataclass
class LookAtActionPerformable(ActionAbstract):
    """
    Lets the robot look at a position.
    """

    target: Pose
    """
    Position at which the robot should look, given as 6D pose
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
    def perform(self) -> None:
        LookingMotion(target=self.target).perform()
