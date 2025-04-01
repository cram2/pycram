from threading import Lock

from .default_process_modules import *
from ..designators.motion_designator import *


class KevinMoveArmJoints(DefaultMoveArmJoints):
    """
    Process module for the simulated Kevin that moves the arm joints of the robot
    """

    def _execute(self, desig: MoveArmJointsMotion):
        # Kevin has very long fingers. It is hard to grasp things with we allow collisions for the gripper when moving
        # for now
        giskard.allow_gripper_collision("gripper")
        DefaultMoveArmJoints._execute(desig)

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