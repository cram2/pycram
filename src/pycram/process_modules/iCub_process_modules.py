from threading import Lock
from typing import Union

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


import yarp
ACK_VOCAB = yarp.createVocab32('a','c','k')
NO_ACK_VOCAB = yarp.createVocab32('n','a','c','k')

def init_yarp_network():
    if not yarp.Network.checkNetwork():
        print("Unable to find a yarp server exiting ...")
        return False

    yarp.Network.init()
    return True

def open_rpc_client_port(port_name):
    handle_port: yarp.RpcClient = yarp.RpcClient()
    if not handle_port.open(port_name):
        print(f"Can't open the port %s correctly" % port_name)
        return False , None
    print(f"Port %s opened correctly" % port_name)
    return True , handle_port


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

    def __init__(self,lock,cmd_port:yarp.RpcClient):
        super().__init__(lock)
        self.cmd_port = cmd_port

    def _execute(self, desig: LookingMotion):
        print("iCub Move Head")
        position_target = desig.target.pose.position
        if self.cmd_port.getOutputCount():

            yarp_bottle_msg: yarp.Bottle = yarp.Bottle()
            yarp_bottle_reply: yarp.Bottle = yarp.Bottle()
            yarp_bottle_msg.addVocab32('l', 'o', 'o', 'k')
            yarp_bottle_msg.addVocab32('3', 'D')

            target_loc: yarp.Bottle = yarp_bottle_msg.addList()
            target_loc.addFloat32(position_target.x)
            target_loc.addFloat32(position_target.y)
            target_loc.addFloat32(position_target.z)
            print(f"command Ready to send to iCub")
            self.cmd_port .write(yarp_bottle_msg, yarp_bottle_reply)
            reply_vocab = yarp_bottle_reply.get(0).asVocab32()

            if reply_vocab == NO_ACK_VOCAB:
                print("NO_ACK")
                return False
            elif reply_vocab == ACK_VOCAB:
                print("ACK")
                return True
            else:
                print("another reply")
                return False


        else:
            print("port is not connected")
            return False



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
        self.yarp_network_state = init_yarp_network()

        # yarp related
        self.gaze_cmd_port_name = "/pycram/gaze/cmd:oi"
        self.action_cmd_port_name = "/pycram/action/cmd:oi"

        self.gaze_client_port = None
        self.action_client_port = None

        if self.yarp_network_state:
            print("yarp network state detected")
            self.config_yarp_ports()
            self.connect_yarp_ports()
        else:
            print("yarp network state not detected")




    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            print('Navigate iCub')
            return iCubNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return iCubNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveHead(self._looking_lock,self.gaze_client_port)
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


    def config_yarp_ports(self)->bool:
        suc, self.gaze_client_port = open_rpc_client_port(self.gaze_cmd_port_name)
        suc2, self.action_client_port = open_rpc_client_port(self.action_cmd_port_name)

        return suc and suc2

    def connect_yarp_ports(self)->bool:
        gaze_connected = yarp.NetworkBase_connect(self.gaze_cmd_port_name, "/iKinGazeCtrl/rpc", "tcp")
        action_connected = yarp.NetworkBase_connect(self.action_cmd_port_name, "/actionsRenderingEngine/cmd:io", "tcp")

        if not gaze_connected:
            print("gaze control port couldn't connect")

        if not action_connected:
            print("action port couldn't connect")

        return gaze_connected and action_connected


    def exit(self):
        self.disconnect_and_remove()


    def disconnect_and_remove(self):
        self.gaze_client_port .interrupt()
        self.gaze_client_port .close()

        self.action_client_port.interrupt()
        self.action_client_port.close()

