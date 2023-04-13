class PlanFailure(Exception):
    """Implementation of plan failures."""

    def __init__(self, *args, **kwargs):
        """Create a new plan failure."""
        Exception.__init__(self, *args, **kwargs)


class FailureDiagnosis(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class LowLevelFailure(FailureDiagnosis):
    """Failure thrown by low-level modules: robot or projection PMs."""

    def __init__(self):
        super().__init__()


class ActionlibActionTimeout(LowLevelFailure):
    """"""

    def __init__(self):
        super().__init__()


class HighLevelFailure(FailureDiagnosis):
    """Failure thrown by high-level modules, i.e. plans."""

    def __init__(self):
        super().__init__()


class DeliveringFailed(HighLevelFailure):
    """Thrown when delivering plan completely gives up."""

    def __init__(self):
        super().__init__()


class ManipulationLowLevelFailure(LowLevelFailure):
    """Thrown when a low-level, i.e. hardware related, failure is detected in a manipulation action."""

    def __init__(self):
        super().__init__()


class EnvironmentManipulationGoalNotReached(ManipulationLowLevelFailure):
    """Thrown when door / drawer opening / closing goal is still not reached."""

    def __init__(self):
        super().__init__()


class EnvironmentManipulationImpossible(HighLevelFailure):
    """Thrown when environment manipulation cannot be achieved."""

    def __init__(self):
        super().__init__()


class EnvironmentUnreachable(HighLevelFailure):
    """Thrown when environment manipulation in collision or unreachable."""

    def __init__(self):
        super().__init__()


class FetchingFailed(HighLevelFailure):
    """Thrown when fetching plan completely gives up."""

    def __init__(self):
        super().__init__()


class GripperLowLevelFailure(LowLevelFailure):
    """Thrown when a failure involving the gripper hardware occurs."""

    def __init__(self):
        super().__init__()


class GripperClosedCompletely(GripperLowLevelFailure):
    """Thrown when the gripper closed completely, despite not being expected to do so (e.g. because it should have
    grasped something)."""

    def __init__(self):
        super().__init__()


class GripperGoalNotReached(GripperLowLevelFailure):
    """Thrown when the gripper does not reach its goal."""

    def __init__(self):
        super().__init__()


class LookingHighLevelFailure(HighLevelFailure):
    """High-level failure produced when looking for an object, i.e. it is not a hardware issue but one relating to
    the looking task, its parameters, and how they relate to the environment."""

    def __init__(self):
        super().__init__()


class ManipulationGoalInCollision(HighLevelFailure):
    """Thrown when executing a manipulation action results in a collision."""

    def __init__(self):
        super().__init__()


class ManipulationGoalNotReached(ManipulationLowLevelFailure):
    """Thrown when after executing the action, goal is still not reached."""

    def __init__(self):
        super().__init__()


class ManipulationPoseUnreachable(ManipulationLowLevelFailure):
    """Thrown when no IK solution can be found."""

    def __init__(self):
        super().__init__()


class NavigationHighLevelFailure(HighLevelFailure):
    """High-level failure produced while navigating the robot, i.e. it is not a hardware issue but one relating to
    the navigation task, its parameters, and how they relate to the environment."""

    def __init__(self):
        super().__init__()


class NavigationGoalInCollision(NavigationHighLevelFailure):
    """Navigation goal cannot be reached because the goal itself is already occupied by some other object."""

    def __init__(self):
        super().__init__()


class NavigationLowLevelFailure(LowLevelFailure):
    """Low-level failure produced while navigating the robot, i.e. some kind of hardware issue."""

    def __init__(self):
        super().__init__()


class NavigationGoalNotReached(NavigationLowLevelFailure):
    """Thrown when the base moved as a result of the navigation action but the goal was not reached."""

    def __init__(self):
        super().__init__()


class NavigationPoseUnreachable(NavigationLowLevelFailure):
    """Thrown when the goal pose for navigation is computed to be unreachable."""

    def __init__(self):
        super().__init__()


class ObjectNowhereToBeFound(HighLevelFailure):
    """Thrown when the robot cannot find an object of a given description in its surroundings."""

    def __init__(self):
        super().__init__()


class ObjectUndeliverable(HighLevelFailure):
    """Thrown when no base positioning can assure a reachable pose to place the object from."""

    def __init__(self):
        super().__init__()


class ObjectUnfetchable(HighLevelFailure):
    """Thrown when no base positioning can assure a reachable pose to grasp the object from."""

    def __init__(self):
        super().__init__()


class ObjectUnreachable(HighLevelFailure):
    """Thrown when no IK found for particular base pose."""

    def __init__(self):
        super().__init__()


class PerceptionLowLevelFailure(LowLevelFailure):
    """Low-level failure produced while perceiving, i.e. some kind of hardware issue."""

    def __init__(self):
        super().__init__()


class PerceptionObjectNotFound(PerceptionLowLevelFailure):
    """Thrown when an attempt to find an object by perception fails -- and this can still be interpreted as the robot
    not looking in the right direction, as opposed to the object being absent."""

    def __init__(self):
        super().__init__()


class PerceptionObjectNotInWorld(PerceptionLowLevelFailure):
    """Thrown when an attempt to find an object by perception fails -- and this is because the object can be assumed
    absent or perhaps is known absent because of the setup of a simulation."""

    def __init__(self):
        super().__init__()


class SearchingFailed(HighLevelFailure):
    """Thrown when searching plan completely gives up."""

    def __init__(self):
        super().__init__()


class TorsoLowLevelFailure(LowLevelFailure):
    """Low-level failure produced while moving the torso, i.e. some kind of hardware issue."""

    def __init__(self):
        super().__init__()


class TorsoGoalNotReached(TorsoLowLevelFailure):
    """Thrown when the torso moved as a result of a torso action but the goal was not reached."""

    def __init__(self):
        super().__init__()


class TorsoGoalUnreachable(TorsoLowLevelFailure):
    """Thrown when the goal for the torso is computed to be unreachable."""

    def __init__(self):
        super().__init__()


class Task(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class Grasping(Task):
    """"""

    def __init__(self):
        super().__init__()


class Looking(Task):
    """"""

    def __init__(self):
        super().__init__()


class ObjectPoseMisestimation(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class SuccessfulCompletion(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class ObjectNotFound(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class LocomotorFailure(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class ArmFailure(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class ObjectLost(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class SensorFailure(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class IllPosedGoalFailure(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class CapabilityAbsenceFailure(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class ReachabilityFailure(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class TorsoFailure(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class ConfigurationNotReached(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class Timeout(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class EndEffectorFailure(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class ObjectUnavailable(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()


class SustainedFailure(PlanFailure):
    """"""

    def __init__(self):
        super().__init__()
