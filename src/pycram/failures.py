from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path

from typing_extensions import TYPE_CHECKING, List, Optional

if TYPE_CHECKING:
    from .datastructures.pose import PoseStamped
    from .description import Link, Joint
    from .world_concepts.world_object import Object
    from .datastructures.enums import JointType, MultiverseAPIName, Arms, StaticJointState, Grasp, DetectionTechnique, \
        ContainerManipulationType
    from .datastructures.world_entity import PhysicalBody
    from .validation.goal_validator import MultiJointPositionGoalValidator
    from .designator import ObjectDesignatorDescription
    from .designators.location_designator import Location


class PlanFailure(Exception):
    """Implementation of plan failures."""

    def __init__(self, *args, **kwargs):
        """Create a new plan failure."""
        Exception.__init__(self, *args, **kwargs)


class KnowledgeNotAvailable(PlanFailure):
    """Thrown when a knowledge source can not provide the information for a query."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class NotALanguageExpression(PlanFailure):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class FailureDiagnosis(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class LowLevelFailure(FailureDiagnosis):
    """Failure thrown by low-level modules: robot or projection PMs."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ActionlibActionTimeout(LowLevelFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class HighLevelFailure(FailureDiagnosis):
    """Failure thrown by high-level modules, i.e. plans."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class SensorMonitoringCondition(PlanFailure):
    """Thrown when a sensor monitoring condition is met."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class DeliveringFailed(HighLevelFailure):
    """Thrown when delivering plan completely gives up."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ManipulationLowLevelFailure(LowLevelFailure):
    """Thrown when a low-level, i.e. hardware related, failure is detected in a manipulation action."""
    robot: Object
    """
    The robot that the manipulation action was performed with.
    """
    arm: List[Arms]
    """
    The arm(s) that the manipulation action was performed with.
    """
    body: PhysicalBody
    """
    The body that the manipulation action was performed on.
    """

    def __init__(self, robot: Object, arms: List[Arms], body: PhysicalBody, *args, **kwargs):
        self.robot = robot
        self.arm = arms
        self.body = body
        super().__init__(*args, **kwargs)


class EnvironmentManipulationGoalNotReached(ManipulationLowLevelFailure):
    """Thrown when door / drawer opening / closing goal is still not reached."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ContainerManipulationError(ManipulationLowLevelFailure, ABC):
    """Thrown when container manipulation fails."""
    container_joint: Joint
    """
    The joint of the container that should be manipulated.
    """
    manipulation_type: ContainerManipulationType
    """
    The type of manipulation that should be performed on the container.
    """

    def __init__(self, robot: Object, arms: List[Arms], body: PhysicalBody, container_joint: Joint,
                 manipulation_type: ContainerManipulationType,
                 *args, **kwargs):
        self.container_joint = container_joint
        self.manipulation_type = manipulation_type
        super().__init__(robot, arms, body,
                         f"manipulation type \"{manipulation_type.name}\" failedContainer {body.name} with joint"
                         f" name {container_joint.name}"
                         f" (current position is {container_joint.position}) and limits {container_joint.limits},"
                         f" using {[arm.name for arm in arms]} arm of {robot.name} robot", *args, **kwargs)


class EnvironmentManipulationImpossible(HighLevelFailure):
    """Thrown when environment manipulation cannot be achieved."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class EnvironmentUnreachable(HighLevelFailure):
    """Thrown when environment manipulation in collision or unreachable."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class FetchingFailed(HighLevelFailure):
    """Thrown when fetching plan completely gives up."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class GripperLowLevelFailure(LowLevelFailure):
    """Thrown when a failure involving the gripper hardware occurs."""
    robot: Object
    """
    The robot that the gripper belongs to.
    """
    arm: Arms
    """
    The arm that the gripper belongs to.
    """

    def __init__(self, robot: Object, arm: Arms, *args, **kwargs):
        self.robot = robot
        self.arm = arm
        super().__init__(*args, **kwargs)


class GripperIsNotOpen(GripperLowLevelFailure):
    """Thrown when the gripper is not open when it should be open."""

    def __init__(self, robot: Object, arm: Arms, *args, **kwargs):
        super().__init__(robot, arm,
                         f"The gripper of arm {arm.name} of robot {robot.name} should be open but is not",
                         *args, **kwargs)


class GripperClosedCompletely(GripperLowLevelFailure):
    """Thrown when the gripper closed completely, despite not being expected to do so (e.g. because it should have
    grasped something)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class GripperGoalNotReached(GripperLowLevelFailure):
    """Thrown when the gripper does not reach its goal."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class GripperOccupied(GripperLowLevelFailure):
    """Thrown when the gripper is occupied by some object."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class LookingHighLevelFailure(HighLevelFailure):
    """High-level failure produced when looking for an object, i.e. it is not a hardware issue but one relating to
    the looking task, its parameters, and how they relate to the environment."""
    robot: Object
    """
    The robot that performed the look at action.
    """
    target: PoseStamped
    """
    The target pose that the robot was supposed to look at.
    """

    def __init__(self, robot: Object, target: PoseStamped, *args, **kwargs):
        self.robot = robot
        self.target = target
        super().__init__(*args, **kwargs)


class LookAtGoalNotReached(LookingHighLevelFailure):
    """Thrown when the look at goal is not reached."""

    def __init__(self, robot: Object, target: PoseStamped, *args, **kwargs):
        super().__init__(robot, target, f"Look at action failed for {robot.name} and target "
                                        f"{target.position.to_list()}{target.orientation.to_list()}", *args, **kwargs)


class ManipulationGoalInCollision(HighLevelFailure):
    """Thrown when executing a manipulation action results in a collision."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ManipulationGoalNotReached(ManipulationLowLevelFailure):
    """Thrown when after executing the action, goal is still not reached."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class RobotInCollision(PlanFailure):
    """Thrown when the robot is in collision with the environment."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class IKError(PlanFailure):
    """Thrown when no inverse kinematics solution could be found"""
    pose: PoseStamped
    """
    The pose for which no IK solution could be found.
    """
    base_frame: str
    """
    The base frame in which the pose was given.
    """
    tip_frame: str
    """
    The robot tip frame that should reach the pose.
    """

    def __init__(self, pose, base_frame, tip_frame):
        self.pose = pose
        self.base_frame = base_frame
        self.tip_frame = tip_frame
        self.message = "Position {} in frame '{}' is not reachable for end effector: '{}'".format(pose, base_frame,
                                                                                                  tip_frame)
        super(IKError, self).__init__(self.message)


class ManipulationPoseUnreachable(ManipulationLowLevelFailure):
    """Thrown when no IK solution can be found."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class NavigationHighLevelFailure(HighLevelFailure):
    """High-level failure produced while navigating the robot, i.e. it is not a hardware issue but one relating to
    the navigation task, its parameters, and how they relate to the environment."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class NavigationGoalInCollision(NavigationHighLevelFailure):
    """Navigation goal cannot be reached because the goal itself is already occupied by some other object."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class NavigationLowLevelFailure(LowLevelFailure):
    """Low-level failure produced while navigating the robot, i.e. some kind of hardware issue."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class NavigationGoalNotReached(NavigationLowLevelFailure):
    """Thrown when the base moved as a result of the navigation action but the goal was not reached."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class NavigationPoseUnreachable(NavigationLowLevelFailure):
    """Thrown when the goal pose for navigation is computed to be unreachable."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectNotVisible(HighLevelFailure):
    """Thrown when the robot cannot see an object of a given description in its surroundings."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectNotFound(HighLevelFailure):
    """Thrown when the robot cannot find an object of a given description in its surroundings."""

    def __init__(self, object_name: str):
        super().__init__(f"Object {object_name} not found")


class LinkNotFound(HighLevelFailure):
    """Thrown when the robot cannot find a link of a given description"""
    link_name: str
    """
    The name of the link that couldn't be found
    """
    of_object: Optional[str]
    """
    The name of the object that the link should belong to
    """

    def __init__(self, link_name: str, of_object: Optional[str] = None):
        self.link_name = link_name
        self.of_object = of_object if of_object else None
        of_object_str = f" of object {of_object}" if of_object else ""
        super().__init__(f"Link {self.link_name} not found" + of_object_str)


class ObjectUndeliverable(HighLevelFailure):
    """Thrown when no base positioning can assure a reachable pose to place the object from."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectPlacingError(HighLevelFailure):
    """Thrown when the placing of the object fails."""
    obj: Object
    """
    The object that should be placed.
    """
    placing_pose: PoseStamped
    """
    The target pose at which the object should be placed.
    """
    robot: Object
    """
    The robot that placed the object.
    """
    arm: Arms
    """
    The robot arm used to place the object.
    """

    def __init__(self, obj: Object, placing_pose: PoseStamped, robot: Object, arm: Arms,
                 *args, **kwargs):
        self.obj = obj
        self.placing_pose = placing_pose
        self.robot = robot
        self.arm = arm
        super().__init__(*args, **kwargs)


class ObjectStillInContact(ObjectPlacingError):
    """Thrown when the object is still in contact with the robot after placing."""
    contact_links: List[Link]
    """
    The links of the robot that are still in contact with the object.
    """

    def __init__(self, obj: Object, contact_links: List[Link], placing_pose: PoseStamped, robot: Object, arm: Arms,
                 *args, **kwargs):
        self.contact_links = contact_links
        contact_link_names = [link.name for link in contact_links]
        super().__init__(obj, placing_pose, robot, arm,
                         f"Object {obj.name} is still in contact with {robot.name}, the contact links are"
                         f"{contact_link_names}, after placing at"
                         f" target pose {placing_pose.position.to_list()}{placing_pose.orientation.to_list()} using"
                         f" {arm.name} arm", *args, **kwargs)


class ObjectNotPlacedAtTargetLocation(ObjectPlacingError):
    """Thrown when the object was not placed at the target location."""

    def __init__(self, obj: Object, placing_pose: PoseStamped, robot: Object, arm: Arms, *args, **kwargs):
        super().__init__(obj, placing_pose, robot, arm,
                         "Object {obj.name} was not placed at target pose {placing_pose.position.to_list()}"
                         f"{placing_pose.orientation.to_list()} using {arm.name} arm of {robot.name}", *args, **kwargs)


class ObjectUnfetchable(HighLevelFailure):
    """Thrown when no base positioning can assure a reachable pose to grasp the object from."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectUnreachable(HighLevelFailure):
    """Thrown when no IK found for particular base pose."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class PerceptionLowLevelFailure(LowLevelFailure):
    """Low-level failure produced while perceiving, i.e. some kind of hardware issue."""
    object_description: ObjectDesignatorDescription
    """
    The object description that was used to search for the object.
    """
    technique: DetectionTechnique
    """
    The detection technique that was used to search for the object.
    """
    region: Optional[Location] = None
    """
    The suggested region in which the object was searched.
    """

    def __init__(self, object_description: ObjectDesignatorDescription, technique: DetectionTechnique,
                 region: Optional[Location] = None, *args, **kwargs):
        self.object_description = object_description
        self.technique = technique
        self.region = region
        super().__init__(*args, **kwargs)


class PerceptionObjectNotFound(PerceptionLowLevelFailure):
    """Thrown when an attempt to find an object by perception fails -- and this can still be interpreted as the robot
    not looking in the right direction, as opposed to the object being absent."""

    def __init__(self, obj_desc: ObjectDesignatorDescription, technique: DetectionTechnique, region: Location,
                 *args, **kwargs):
        super().__init__(obj_desc, technique, region,
                         f"object described by {obj_desc} not found using {technique.name} technique in region"
                         f" {region}", *args, **kwargs)


class PerceptionObjectNotInWorld(PerceptionLowLevelFailure):
    """Thrown when an attempt to find an object by perception fails -- and this is because the object can be assumed
    absent or perhaps is known absent because of the setup of a simulation."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class SearchingFailed(HighLevelFailure):
    """Thrown when searching plan completely gives up."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class TorsoLowLevelFailure(LowLevelFailure):
    """Low-level failure produced while moving the torso, i.e. some kind of hardware issue."""
    goal_validator: Optional[MultiJointPositionGoalValidator] = None
    """
    The goal validator that was used to check if the goal was reached.
    """

    def __init__(self, goal_validator: Optional[MultiJointPositionGoalValidator] = None, *args, **kwargs):
        self.goal_validator = goal_validator
        if goal_validator:
            super().__init__(goal_validator.goal_not_achieved_message, *args, **kwargs)
        else:
            super().__init__(*args, **kwargs)


class TorsoGoalNotReached(TorsoLowLevelFailure):
    """Thrown when the torso moved as a result of a torso action but the goal was not reached."""


class TorsoGoalUnreachable(TorsoLowLevelFailure):
    """Thrown when the goal for the torso is computed to be unreachable."""


class Task(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class Grasping(Task):
    """"""
    obj: Object
    """
    The object to be grasped.
    """
    robot: Object
    """
    The robot that should grasp the object.
    """
    arm: Arms
    """
    The arm used to grasp the object.
    """
    grasp: Optional[Grasp]
    """
    The grasp type used to grasp the object.
    """

    def __init__(self, obj: Object, robot: Object, arm: Arms, grasp: Optional[Grasp] = None, *args, **kwargs):
        self.obj = obj
        self.robot = robot
        self.arm = arm
        self.grasp = grasp
        super().__init__(*args, **kwargs)


class ObjectNotGraspedError(Grasping):
    def __init__(self, obj: Object, robot: Object, arm: Arms, grasp = None, *args, **kwargs):
        grasp_str = f"using {grasp} grasp" if grasp else ""
        super().__init__(obj, robot, arm, grasp, f"object {obj.name} was not grasped by {arm.name} arm" + grasp_str,
                         *args, **kwargs)


class Looking(Task):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectPoseMissestimation(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class SuccessfulCompletion(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class LocomotorFailure(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ArmFailure(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectLost(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class SensorFailure(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class IllPosedGoalFailure(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class CapabilityAbsenceFailure(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ReachabilityFailure(PlanFailure):
    """"""
    obj: Object
    """
    The object that should be reachable.
    """
    robot: Object
    """
    The robot that should reach the object.
    """
    arm: Arms
    """
    The arm that should reach the object.
    """
    grasp: Optional[Grasp] = None
    """
    The grasp/gripper orientation that should be used to reach the object.
    """

    def __init__(self, obj: Object, robot: Object, arm: Arms, grasp: Grasp, *args, **kwargs):
        self.obj = obj
        self.robot = robot
        self.arm = arm
        self.grasp = grasp
        super().__init__(*args, **kwargs)


class ObjectNotInGraspingArea(ReachabilityFailure):
    def __init__(self, obj: Object, robot: Object, arm: Arms, grasp, *args, **kwargs):
        super().__init__(obj, robot, arm, grasp,
                         f"object {obj.name} is not in the grasping area of robot {robot.name} using {arm.name} arm and"
                         f" {grasp} grasp", *args, **kwargs)


class TorsoFailure(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ConfigurationNotReached(PlanFailure):
    """"""
    goal_validator: MultiJointPositionGoalValidator
    """
    The goal validator that was used to check if the goal was reached.
    """
    configuration_type: StaticJointState
    """
    The configuration type that should be reached.
    """

    def __init__(self, goal_validator: MultiJointPositionGoalValidator, configuration_type: StaticJointState,
                 *args, **kwargs):
        self.goal_validator = goal_validator
        self.configuration_type = configuration_type
        super().__init__(f"configuration_type: {configuration_type.name},"
                         f" {goal_validator.goal_not_achieved_message}",
                         *args, **kwargs)


class Timeout(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class EndEffectorFailure(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectUnavailable(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class SustainedFailure(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ReasoningError(PlanFailure):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class CollisionError(PlanFailure):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class NavigationGoalNotReachedError(PlanFailure):
    """
    Thrown when the navigation goal is not reached.
    """
    current_pose: PoseStamped
    """
    The current pose of the robot.
    """
    goal_pose: PoseStamped
    """
    The goal pose of the robot.
    """

    def __init__(self, current_pose: PoseStamped, goal_pose: PoseStamped, *args, **kwargs):
        self.current_pose = current_pose
        self.goal_pose = goal_pose
        super().__init__(f"Navigation goal not reached. Current pose: {current_pose}, goal pose: {goal_pose}",
                         *args, **kwargs)


class ToolPoseNotReachedError(PlanFailure):
    """
    Thrown when the tool pose is not reached.
    """
    current_pose: PoseStamped
    """
    The current pose of the tool.
    """
    goal_pose: PoseStamped
    """
    The goal pose of the tool.
    """

    def __init__(self, current_pose: PoseStamped, goal_pose: PoseStamped, *args, **kwargs):
        self.current_pose = current_pose
        self.goal_pose = goal_pose
        super().__init__(f"Tool pose not reached. Current pose: {current_pose}, goal pose: {goal_pose}",
                         *args, **kwargs)


"""
The following exceptions are used in the PyCRAM framework to handle errors related to the world and the objects in it.
They are usually related to a bug in the code or a misuse of the framework (e.g. logical errors in the code).
"""


class MultiverseFailedAPIResponse(Exception):
    """
    Exception raised when a Multiverse API call fails.
    """
    api_response: List[str]
    """
    The response of the API call that failed.
    """
    api_name: MultiverseAPIName
    """
    The name of the API that failed.
    """
    def __init__(self, api_response: List[str], api_name: MultiverseAPIName, *args, **kwargs):
        self.api_response = api_response
        self.api_name = api_name
        super().__init__(f"{api_name} api request with arguments {args} and keyword arguments {kwargs}"
                         f" failed with response {api_response}")


class ProspectionObjectNotFound(KeyError):
    """
    Exception raised when an object was not found in the prospection world.
    """
    obj: Object
    """
    The object that was not found in the prospection world.
    """
    def __init__(self, obj: Object):
        self.obj = obj
        super().__init__(f"The given object {obj.name} is not in the prospection world.")


class ObjectAlreadyExists(Exception):
    """
    Exception raised when an object with the same name already exists in the world.
    """
    obj: Object
    """
    The object that already exists in the world.
    """
    def __init__(self, obj: Object):
        self.obj = obj
        super().__init__(f"An object with the name {obj.name} already exists in the world.")


class ObjectDescriptionNotFound(KeyError):
    """
    Exception raised when the description of an object was not found.
    """
    object_name: str
    """
    The name of the object whose description was not found.
    """
    path: str
    """
    The path of the object whose description was not found.
    """
    extension: str
    """
    The description extension of the object whose description was not found.
    """
    def __init__(self, object_name: str, path: str, extension: str):
        self.object_name = object_name
        self.path = path
        self.extension = extension
        super().__init__(f"{object_name} with path {path} and extension {extension} is not in supported extensions, and"
                         f" the description data was not found on the ROS parameter server")


class WorldMismatchErrorBetweenAttachedObjects(Exception):
    """
    Exception raised when two objects that are attached to each other have a mismatch in the world they belong to.
    """
    obj_1: Object
    """
    The first object that has a mismatch in the world.
    """
    obj_2: Object
    """
    The second object that has a mismatch in the world.
    """
    def __init__(self, obj_1: 'Object', obj_2: 'Object'):
        self.obj_1 = obj_1
        self.obj_2 = obj_2
        super().__init__(f"World mismatch between the attached objects {obj_1.name} and {obj_2.name},"
                         f"obj_1.world: {obj_1.world}, obj_2.world: {obj_2.world}")


class ObjectFrameNotFoundError(KeyError):
    """
    Exception raised when a tf frame of an object is not found.
    """
    frame_name: str
    """
    The name of the frame that was not found.
    """
    def __init__(self, frame_name: str):
        self.frame_name = frame_name
        super().__init__(f"Frame {frame_name} does not belong to any of the objects in the world.")


class MultiplePossibleTipLinks(Exception):
    """
    Exception raised when multiple tip links are found for an object.
    """
    object_name: str
    """
    The name of the object that has multiple tip links.
    """
    start_link: str
    """
    The start link of the object that has multiple tip links.
    """
    tip_links: List[str]
    """
    The list of tip links that are found for the object.
    """
    def __init__(self, object_name: str, start_link: str, tip_links: List[str]):
        self.object_name = object_name
        self.start_link = start_link
        self.tip_links = tip_links
        super().__init__(f"Multiple possible tip links found for object {object_name} with start link {start_link}:"
                         f" {tip_links}")


class UnsupportedFileExtension(Exception):
    """
    Exception raised when an object mesh/description has an unsupported file extension.
    """
    object_name: str
    """
    The name of the object that has an unsupported file extension.
    """
    path: str
    """
    The path of the object description/mesh that has an unsupported file extension.
    """
    extension: str
    """
    The unsupported file extension of the object description/mesh.
    """
    def __init__(self, object_name: str, path: str):
        self.object_name = object_name
        self.path = path
        self.extension = Path(path).suffix
        super().__init__(f"Unsupported file extension for object {object_name} with path {path}"
                         f"and extension {self.extension}")


class ObjectDescriptionUndefined(Exception):
    """
    Exception raised when the given object description type is not defined or couldn't be resolved to a known type.
    """
    object_name: str
    """
    The name of the object that has an undefined description.
    """
    def __init__(self, object_name: str):
        self.object_name = object_name
        super().__init__(f"Object description for object {object_name} is not defined, either a path or a description"
                         f"object should be provided.")


class UnsupportedJointType(Exception):
    """
    Exception raised when an unsupported joint type is used.
    """
    joint_type: JointType
    """
    The unsupported joint type that was used.
    """
    def __init__(self, joint_type: JointType):
        self.joint_type = joint_type
        super().__init__(f"Unsupported joint type: {joint_type}")


class LinkHasNoGeometry(Exception):
    """
    Exception raised when a link has no geometry (i.e. no visual or collision elements).
    """
    link_name: str
    """
    The name of the link that has no geometry.
    """
    def __init__(self, link_name: str):
        self.link_name = link_name
        super().__init__(f"Link {link_name} has no geometry.")


class LinkGeometryHasNoMesh(Exception):
    """
    Exception raised when a link geometry has no mesh or is not of type mesh.
    """
    link_name: str
    """
    The name of the link that has no mesh.
    """
    geometry_type: str
    """
    The type of the link geometry.
    """
    def __init__(self, link_name: str, geometry_type: str):
        self.link_name = link_name
        self.geometry_type = geometry_type
        super().__init__(f"Link {link_name} geometry with type {geometry_type} has no mesh.")
