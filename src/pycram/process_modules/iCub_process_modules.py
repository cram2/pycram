from threading import Lock
from typing_extensions import Any

import actionlib

from .. import world_reasoning as btr
import numpy as np

from ..process_module import ProcessModule, ProcessModuleManager
from ..external_interfaces.ik import request_ik
from ..ros.logging import logdebug
from ..utils import _apply_ik
from ..local_transformer import LocalTransformer
from ..designators.object_designator import ObjectDesignatorDescription
from ..designators.motion_designator import MoveMotion, LookingMotion, \
    DetectingMotion, MoveTCPMotion, MoveArmJointsMotion, WorldStateDetectingMotion, MoveJointsMotion, \
    MoveGripperMotion, OpeningMotion, ClosingMotion
from ..robot_description import RobotDescription
from ..datastructures.world import World
from ..world_concepts.world_object import Object
from ..datastructures.pose import Pose
from ..datastructures.enums import JointType, ObjectType, Arms, ExecutionType
from ..external_interfaces import giskard
from ..external_interfaces.robokudo import *



class iCubNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        print("iCubNavigate")


class iCubMoveHead(ProcessModule):
    """
        This process module moves the head to look at a specific point in the world coordinate frame.
        This point can either be a position or an object.
        """

    def _execute(self, desig: LookingMotion):
        print("iCub Move Head")


class iCubMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion):
        print("iCub Move Gripper")


class iCubDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion):
        print("iCub Detect")

class iCubMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion):
        print("iCub Move TCP")


class iCubMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion):
        print("iCub Move Arm Joints")

class iCubMoveJoints(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def _execute(self, desig: MoveJointsMotion):
        print("iCub Move Joints")


class iCubWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        print("iCub World State Detecting")


class iCubOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        print("iCub Open")


class iCubClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        print("iCub Close")



###########################################################
########## Process Modules for the Real iCub ##############
###########################################################


class iCubNavigationReal(ProcessModule):
    """
    Process module for the real PR2 that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        print("iCub Navigate Real")


class iCubMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion):
        print("iCub Move Head Real")

class iCubDetectingReal(ProcessModule):
    """
    Process Module for the real Pr2 that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion) -> Any:
        print("iCub Detecting Real")


class iCubMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real PR2 while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        print("iCub Move TCP Real")


class iCubMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real iCub to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        print("iCub Move Arm Joints Real")


class iCubMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        print("iCub Move Joints Real")


class iCubMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real PR2, gripper uses an action server for this instead of giskard 
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        print("iCub Move Gripper Real")


class iCubOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        print("iCub Open Real")


class iCubCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        print("iCub Close Real")


class ICubManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("iCub")
        self._navigate_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            print('Navigate iCub')
            return iCubNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if (ProcessModuleManager.execution_type == ExecutionType.SIMULATED or
                ProcessModuleManager.execution_type == ExecutionType.REAL):
            return iCubWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubClose(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubCloseReal(self._close_lock)
