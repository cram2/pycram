from threading import Lock

# from ..external_interfaces.robokudo import query
from .default_process_modules import *
from ..designators.motion_designator import *


class KevinNavigate(DefaultNavigation):
    """
    Process module for the simulated Kevin that sends a cartesian goal to the robot to move the robot base
    """
    pass


class KevinMoveHead(DefaultMoveHead):
    """
    Process module for the simulated Kevin that moves the head such that it looks at the given position
    """
    pass


class KevinMoveGripper(DefaultMoveGripper):
    """
    Process module for the simulated Kevin that opens or closes the gripper
    """
    pass


class KevinDetecting(DefaultDetecting):
    """
    Process Module for the simulated Kevin that tries to detect an object fitting the given object description
    """
    pass


class KevinMoveTCP(DefaultMoveTCP):
    """
    Process module for the simulated Kevin that moves the tool center point of the robot
    """
    pass


class KevinMoveArmJoints(DefaultMoveArmJoints):
    """
    Process module for the simulated Kevin that moves the arm joints of the robot
    """

    def _execute(self, desig: MoveArmJointsMotion):
        # Kevin has very long fingers. It is hard to grasp things with we allow collisions for the gripper when moving
        # for now
        giskard.allow_gripper_collision("gripper")
        DefaultMoveArmJoints._execute(desig)


class KevinMoveJoints(DefaultMoveJoints):
    """
    Process module for the simulated Kevin that moves any joint of the robot
    """
    pass


class KevinWorldStateDetecting(DefaultWorldStateDetecting):
    """
    Process Module for the simulated Kevin that tries to detect an object using the world state
    """
    pass


class KevinOpen(DefaultOpen):
    """
    Process module for the simulated Kevin that opens an already grasped container
    """
    pass


class KevinClose(DefaultClose):
    """
    Process module for the simulated Kevin that closes an already grasped container
    """
    pass


###########################################################
########## Process Modules for the Real Kevin ###########
###########################################################

# We never had the real robot or worked on it


class KevinManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "kevin"
        self._navigate_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            raise NotImplemented
            # return KevinNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return KevinMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            raise NotImplemented
            # return KevinMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            raise NotImplemented
            # return KevinDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            raise NotImplemented
            # return KevinMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveArmJointsReal(self._move_arm_joints_lock)
            # return KevinMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED or ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveJointsReal(self._move_arm_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            raise NotImplemented
            # return KevinMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return KevinOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            raise NotImplemented
            # return KevinOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return KevinClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            raise NotImplemented
            # return KevinCloseReal(self._close_lock)
