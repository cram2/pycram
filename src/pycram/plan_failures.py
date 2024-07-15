class PlanFailure(Exception):
    """Implementation of plan failures."""

    def __init__(self, *args, **kwargs):
        """Create a new plan failure."""
        Exception.__init__(self, *args, **kwargs)


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


class DeliveringFailed(HighLevelFailure):
    """Thrown when delivering plan completely gives up."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ManipulationLowLevelFailure(LowLevelFailure):
    """Thrown when a low-level, i.e. hardware related, failure is detected in a manipulation action."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class EnvironmentManipulationGoalNotReached(ManipulationLowLevelFailure):
    """Thrown when door / drawer opening / closing goal is still not reached."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class GripperClosedCompletely(GripperLowLevelFailure):
    """Thrown when the gripper closed completely, despite not being expected to do so (e.g. because it should have
    grasped something)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class GripperGoalNotReached(GripperLowLevelFailure):
    """Thrown when the gripper does not reach its goal."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class LookingHighLevelFailure(HighLevelFailure):
    """High-level failure produced when looking for an object, i.e. it is not a hardware issue but one relating to
    the looking task, its parameters, and how they relate to the environment."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ManipulationGoalInCollision(HighLevelFailure):
    """Thrown when executing a manipulation action results in a collision."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ManipulationGoalNotReached(ManipulationLowLevelFailure):
    """Thrown when after executing the action, goal is still not reached."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class IKError(PlanFailure):
    """Thrown when no inverse kinematics solution could be found"""
    def __init__(self, pose, base_frame, tip_frame):
        self.message = "Position {} in frame '{}' is not reachable for end effector: '{}'".format(pose, base_frame, tip_frame)
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


class ObjectNowhereToBeFound(HighLevelFailure):
    """Thrown when the robot cannot find an object of a given description in its surroundings."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectUndeliverable(HighLevelFailure):
    """Thrown when no base positioning can assure a reachable pose to place the object from."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class PerceptionObjectNotFound(PerceptionLowLevelFailure):
    """Thrown when an attempt to find an object by perception fails -- and this can still be interpreted as the robot
    not looking in the right direction, as opposed to the object being absent."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class TorsoGoalNotReached(TorsoLowLevelFailure):
    """Thrown when the torso moved as a result of a torso action but the goal was not reached."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class TorsoGoalUnreachable(TorsoLowLevelFailure):
    """Thrown when the goal for the torso is computed to be unreachable."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class Task(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class Grasping(Task):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class Looking(Task):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectPoseMisestimation(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class SuccessfulCompletion(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ObjectNotFound(PlanFailure):
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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class TorsoFailure(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ConfigurationNotReached(PlanFailure):
    """"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


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
    def __init__(*args, **kwargs):
        super().__init__(*args, **kwargs)


class CollisionError(PlanFailure):
    def __init__(*args, **kwargs):
        super().__init__(*args, **kwargs)
