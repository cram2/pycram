import math
from .default_process_modules import DefaultMoveJoints, DefaultMoveArmJoints, DefaultMoveTCP, DefaultNavigation, \
    DefaultMoveHead, DefaultDetecting, DefaultMoveGripper

from ..failures import RobotNotInitialized
from ..process_module import ProcessModule, ProcessModuleManager
from ..ros.logging import logerr
from ..designators.motion_designator import MoveMotion, LookingMotion, \
    DetectingMotion, MoveTCPMotion, MoveArmJointsMotion, WorldStateDetectingMotion, MoveJointsMotion, \
    MoveGripperMotion
from ..robot_description import RobotDescription
from ..datastructures.world import World
from ..datastructures.enums import Arms, ExecutionType, GripperState
from ..external_interfaces.robokudo import *

try:
    from pycram.external_interfaces.yarp_networking import *
except ImportError:
    logwarn("Yarp Was Not Found. You can't use iCub. Check yarp installation")
    yarp = None
    pass

def update_hand(hand_values,ctp_port):
    yarp_bottle_msg: yarp.Bottle = yarp.Bottle()
    yarp_bottle_reply: yarp.Bottle = yarp.Bottle()
    yarp_bottle_msg.addVocab32('c', 't', 'p', 'q')
    yarp_bottle_msg.addVocab32('t', 'i', 'm', 'e')
    yarp_bottle_msg.addFloat32(1.5)
    yarp_bottle_msg.addVocab32('o', 'f', 'f')
    yarp_bottle_msg.addInt32(7)
    yarp_bottle_msg.addVocab32('p', 'o', 's')
    target_loc: yarp.Bottle = yarp_bottle_msg.addList()
    for i in hand_values:
        target_loc.addFloat32(i)


    ctp_port.write(yarp_bottle_msg, yarp_bottle_reply)
    reply_vocab = yarp_bottle_reply.get(0).asVocab32()

    if reply_vocab == NO_ACK_VOCAB:
        loginfo("Received: NO_ACK")
        return False
    elif reply_vocab == ACK_VOCAB:
        loginfo("Received: ACK")
        return True
    else:
        loginfo("Received: another reply")
        return False

def update_part(state_port,ctp_port, joint_to_change_idx, joints_to_change_pos):
    if len(joint_to_change_idx):
        part_state: yarp.Bottle = state_port.read(shouldWait=True)
        part_new_states = []
        for i in range(part_state.size()):
            part_new_states.append(part_state.get(i).asFloat32())

        for i in range(len(joint_to_change_idx)):
            part_new_states[joint_to_change_idx[i]] = math.degrees(joints_to_change_pos[i])

        yarp_bottle_msg: yarp.Bottle = yarp.Bottle()
        yarp_bottle_reply: yarp.Bottle = yarp.Bottle()
        yarp_bottle_msg.addVocab32('c', 't', 'p', 'q')
        yarp_bottle_msg.addVocab32('t', 'i', 'm', 'e')
        yarp_bottle_msg.addFloat32(1.5)
        yarp_bottle_msg.addVocab32('o', 'f', 'f')
        yarp_bottle_msg.addInt32(0)
        yarp_bottle_msg.addVocab32('p', 'o', 's')
        target_loc: yarp.Bottle = yarp_bottle_msg.addList()
        for i in part_new_states:
            target_loc.addFloat32(i)

        ctp_port.write(yarp_bottle_msg, yarp_bottle_reply)
        reply_vocab = yarp_bottle_reply.get(0).asVocab32()

        if reply_vocab == NO_ACK_VOCAB:
            loginfo("Received: NO_ACK")
            return False
        elif reply_vocab == ACK_VOCAB:
            loginfo("Received: ACK")
            return True
        else:
            loginfo("Received: another reply")
            return False

HAND_CLOSED = [60.0,60.0,0,85.0,20,85,20,85,85]
HAND_OPENED = [0   ,0   ,0,0   ,0 ,0 ,0 ,0 ,0]
class iCubNavigationReal(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        loginfo("iCubNavigationReal function. No implementation ATM")


class iCubMoveHeadReal(ProcessModule):
    """
        This process module moves the head to look at a specific point in the world coordinate frame.
        This point can either be a position or an object.
        """

    def __init__(self,lock,cmd_port:yarp.RpcClient):
        super().__init__(lock)
        self.cmd_port = cmd_port

    def _execute(self, desig: LookingMotion):
        loginfo("iCubMoveHeadReal")
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
            self.cmd_port .write(yarp_bottle_msg, yarp_bottle_reply)
            reply_vocab = yarp_bottle_reply.get(0).asVocab32()

            if reply_vocab == NO_ACK_VOCAB:
                loginfo("Received: NO_ACK")
                return False
            elif reply_vocab == ACK_VOCAB:
                loginfo("Received: ACK")
                return True
            else:
                loginfo("Received: another reply")
                return False


        else:
            logwarn("control port is not connected")
            return False



class iCubMoveGripperReal(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """
    def __init__(self, lock,
                 state_ports : [yarp.BufferedPortBottle],
                 ctp_ports: [yarp.RpcClient]):
        super().__init__(lock)
        self.state_ports = state_ports
        self.ctp_ports = ctp_ports
        loginfo("iCubMoveGripperReal initialized")


    def _execute(self, desig: MoveGripperMotion):
        gripper = desig.gripper
        required_status = desig.motion
        if gripper == Arms.RIGHT:
            if required_status == GripperState.CLOSE:
                update_hand(HAND_CLOSED,self.ctp_ports[0])
            else:
                update_hand(HAND_OPENED, self.ctp_ports[0])

        elif gripper == Arms.LEFT:
            if required_status == GripperState.CLOSE:
                update_hand(HAND_CLOSED, self.ctp_ports[1])
            else:
                update_hand(HAND_OPENED, self.ctp_ports[1])

        elif gripper ==Arms.BOTH:
            if required_status == GripperState.CLOSE:
                update_hand(HAND_CLOSED, self.ctp_ports[0])
                update_hand(HAND_CLOSED, self.ctp_ports[1])
            else:
                update_hand(HAND_OPENED, self.ctp_ports[0])
                update_hand(HAND_OPENED, self.ctp_ports[1])
        else:
            logwarn ("undefined arm")


class iCubDetectingReal(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion):
        loginfo("iCubDetectingReal. No Implementation ATM")

class iCubMoveTCPReal(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def __init__(self, lock, cmd_port: yarp.RpcClient):
        super().__init__(lock)
        self.cmd_port = cmd_port

    def _execute(self, desig: MoveTCPMotion):
        loginfo("iCubMoveTCPReal")
        position_target = desig.target.position
        if self.cmd_port.getOutputCount():

            yarp_bottle_msg: yarp.Bottle = yarp.Bottle()
            yarp_bottle_reply: yarp.Bottle = yarp.Bottle()
            yarp_bottle_msg.addVocab32('t', 'o', 'u', 'c')

            target_loc: yarp.Bottle = yarp_bottle_msg.addList()
            target_loc.addFloat32(position_target.x)
            target_loc.addFloat32(position_target.y)
            target_loc.addFloat32(position_target.z)

            yarp_bottle_msg.addString("side")
            if desig.arm == Arms.LEFT:
                yarp_bottle_msg.addString("left")
            elif desig.arm == Arms.RIGHT:
                yarp_bottle_msg.addString("right")

            yarp_bottle_msg.addString("still")

            self.cmd_port.write(yarp_bottle_msg, yarp_bottle_reply)
            reply_vocab = yarp_bottle_reply.get(0).asVocab32()

            if reply_vocab == NO_ACK_VOCAB:
                loginfo("Received: NO_ACK")
                return False
            elif reply_vocab == ACK_VOCAB:
                loginfo("Received: ACK")
                return True
            else:
                loginfo("Received: another reply")
                return False


        else:
            logwarn("port is not connected")
            return False




class iCubMoveArmJointsReal(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def __init__(self, lock,
                 state_ports : [yarp.BufferedPortBottle],
                 ctp_ports: [yarp.RpcClient]):
        super().__init__(lock)
        self.state_ports = state_ports
        self.ctp_ports = ctp_ports
        loginfo("iCubMoveArmJointsReal initialized")


    def _execute(self, desig: MoveArmJointsMotion):

        right_arm_to_change_joints = []
        left_arm_to_change_joints = []

        right_arm_to_change_joints_states = []
        left_arm_to_change_joints_states = []

        right_arm_to_change = desig.right_arm_poses
        left_arm_to_change = desig.left_arm_poses

        if right_arm_to_change is not None:
            for joint_mame , joint_pose in right_arm_to_change.items():
                part_name,part_index,joint_index = RobotDescription.current_robot_description.get_actuated_joint_indices(joint_mame)
                if part_index is not None:
                    if part_index == 1:
                        right_arm_to_change_joints.append(joint_index)
                        right_arm_to_change_joints_states.append(joint_pose)
                    else:
                        logwarn("error in joint name. (Not part of right arm chain)")
                else:
                    logwarn("error in joint name")
        if left_arm_to_change is not None:
            for joint_mame, joint_pose in left_arm_to_change.items():
                part_name,part_index, joint_index = RobotDescription.current_robot_description.get_actuated_joint_indices(
                    joint_mame)
                if part_index is not None:
                    if part_index == 2:
                        left_arm_to_change_joints.append(joint_index)
                        left_arm_to_change_joints_states.append(joint_pose)
                    else:
                        logwarn("error in joint name. (Not part of left arm chain)")
                else:
                    logwarn("error in joint name")


        update_part(self.state_ports[0],
                    self.ctp_ports[0],
                    right_arm_to_change_joints,
                    right_arm_to_change_joints_states)

        update_part(self.state_ports[1],
                    self.ctp_ports[1],
                    left_arm_to_change_joints,
                    left_arm_to_change_joints_states)



class iCubMoveJointsReal(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def __init__(self, lock,
                 state_ports : [yarp.BufferedPortBottle],
                 ctp_ports: [yarp.RpcClient]):
        super().__init__(lock)
        self.state_ports = state_ports
        self.ctp_ports = ctp_ports
        loginfo("iCubMoveJointsReal initialized")



    def _execute(self, desig: MoveJointsMotion):
        torso_to_change_joints = []
        right_arm_to_change_joints = []
        left_arm_to_change_joints = []

        torso_to_change_joints_states = []
        right_arm_to_change_joints_states = []
        left_arm_to_change_joints_states = []

        to_change_joints = desig.names
        to_change_states = desig.positions

        index = 0
        for joint_mame in to_change_joints:
            part_name,part_index,joint_index = RobotDescription.current_robot_description.get_actuated_joint_indices(joint_mame)
            if part_index is not None:
                if part_index == 0:
                    torso_to_change_joints.append(joint_index)
                    torso_to_change_joints_states.append(to_change_states[index])
                elif part_index == 1:
                    right_arm_to_change_joints.append(joint_index)
                    right_arm_to_change_joints_states.append(to_change_states[index])
                elif part_index == 2:
                    left_arm_to_change_joints.append(joint_index)
                    left_arm_to_change_joints_states.append(to_change_states[index])
                else:
                    logwarn("error in index")

            index += 1

        update_part(self.state_ports[0],
                    self.ctp_ports[0],
                    torso_to_change_joints,
                    torso_to_change_joints_states)

        update_part(self.state_ports[1],
                    self.ctp_ports[1],
                    right_arm_to_change_joints,
                    right_arm_to_change_joints_states)

        update_part(self.state_ports[2],
                    self.ctp_ports[2],
                    left_arm_to_change_joints,
                    left_arm_to_change_joints_states)





class iCubWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.obj_type == obj_type, World.current_world.objects))[0]




###########################################################
######## Process Modules for the simulated iCub ###########
###########################################################


class iCubNavigation(DefaultNavigation):
    """
    Process module for the real PR2 that sends a cartesian goal to giskard to move the robot base
    """
    ...


class iCubMoveHead(DefaultMoveHead):
    """
        This process module moves the head to look at a specific point in the world coordinate frame.
        This point can either be a position or an object.
    """

    ...


class iCubDetecting(DefaultDetecting):
    """
    Process Module for the real Pr2 that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    ...


class iCubMoveTCP(DefaultMoveTCP):
    """
    Moves the tool center point of the real PR2 while avoiding all collisions
    """

    ...


class iCubMoveArmJoints(DefaultMoveArmJoints):
    """
    Moves the arm joints of the real iCub to the given configuration while avoiding all collisions
    """
    ...


class iCubMoveJoints(DefaultMoveJoints):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """
    ...


class iCubMoveGripper(DefaultMoveGripper):
    """
    Opens or closes the gripper of the real PR2, gripper uses an action server for this instead of giskard 
    """

    ...


class ICubManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("icub")
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

        self.initialized = False

        if yarp is None:
            logwarn("Yarp not Found. ICubManager wasn't initialized correctly")
            return

        self.yarp_network_state = init_yarp_network()

        # yarp related
        self.robot_name_yarp = "icubSim"
        self.gaze_cmd_port_name = "/pycram/gaze/cmd:oi"
        self.action_cmd_port_name = "/pycram/action/cmd:oi"

        self.ctp_torso_client_port_name = "/pycram/ctp/torso:oi"
        self.ctp_right_arm_client_port_name = "/pycram/ctp/right_arm:oi"
        self.ctp_left_arm_client_port_name = "/pycram/ctp/left_arm:oi"

        self.state_torso_port_name = "/pycram/state/torso:i"
        self.state_right_arm_port_name = "/pycram/state/right_arm:i"
        self.state_left_arm_port_name = "/pycram/state/left_arm:i"

        self.gaze_client_port = None
        self.action_client_port = None

        self.ctp_torso_client_port = None
        self.ctp_right_arm_client_port = None
        self.ctp_left_arm_client_port = None

        self.state_torso_port =  yarp.BufferedPortBottle()
        self.state_right_arm_port =  yarp.BufferedPortBottle()
        self.state_left_arm_port =  yarp.BufferedPortBottle()

        if self.yarp_network_state:
            loginfo("yarp network state detected")
            self.config_yarp_ports()
            self.connect_yarp_ports()
            self.initialized = True
        else:
            logwarn("yarp network state not detected. ICubManager wasn't initialized correctly")


    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            if not self.initialized:
                raise RobotNotInitialized("icub")
            return iCubNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            if not self.initialized:
                raise RobotNotInitialized("icub")
            return iCubMoveHeadReal(self._looking_lock,self.gaze_client_port)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            if not self.initialized:
                raise RobotNotInitialized("icub")
            return iCubDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            if not self.initialized:
                raise RobotNotInitialized("icub")
            return iCubMoveTCPReal(self._move_tcp_lock,self.action_client_port)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveArmJoints(self._move_arm_joints_lock)

        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            if not self.initialized:
                raise RobotNotInitialized("icub")
            return iCubMoveArmJointsReal(self._move_arm_joints_lock,
                                     [self.state_right_arm_port, self.state_left_arm_port],
                                     [self.ctp_right_arm_client_port,self.ctp_left_arm_client_port])

    def world_state_detecting(self):
        if (ProcessModuleManager.execution_type == ExecutionType.SIMULATED or
                ProcessModuleManager.execution_type == ExecutionType.REAL):
            return iCubWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveJoints(self._move_joints_lock)

        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            if not self.initialized:
                raise RobotNotInitialized()
            return iCubMoveJointsReal(self._move_joints_lock,
                                  [self.state_torso_port,self.state_right_arm_port,self.state_left_arm_port],
                                  [self.ctp_torso_client_port,self.ctp_right_arm_client_port,self.ctp_left_arm_client_port])

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return iCubMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            if not self.initialized:
                raise RobotNotInitialized()
            return iCubMoveGripperReal(self._move_gripper_lock,
                                     [self.state_right_arm_port, self.state_left_arm_port],
                                     [self.ctp_right_arm_client_port,self.ctp_left_arm_client_port])

    def open(self):
        logwarn("iCub doesn't perform open action from here")

    def close(self):
        logwarn("iCub doesn't perform close action from here")


    def config_yarp_ports(self)->bool:
        suc, self.gaze_client_port = open_rpc_client_port(self.gaze_cmd_port_name)
        if not suc:
            logerr(f"Failed to open {self.gaze_cmd_port_name}")
            return False
        suc, self.action_client_port = open_rpc_client_port(self.action_cmd_port_name)
        if not suc:
            logerr(f"Failed to open {self.action_cmd_port_name}")
            return False

        suc, self.ctp_torso_client_port = open_rpc_client_port(self.ctp_torso_client_port_name)
        if not suc:
            logerr(f"Failed to open {self.ctp_torso_client_port_name}")
            return False

        suc, self.ctp_right_arm_client_port = open_rpc_client_port(self.ctp_right_arm_client_port_name)
        if not suc:
            logerr(f"Failed to open {self.ctp_right_arm_client_port_name}")
            return False

        suc, self.ctp_left_arm_client_port = open_rpc_client_port(self.ctp_left_arm_client_port_name)
        if not suc:
            logerr(f"Failed to open {self.ctp_left_arm_client_port_name}")
            return False

        suc, self.state_torso_port = open_buffered_bottle_port(self.state_torso_port_name)
        if not suc:
            logerr(f"Failed to open {self.state_torso_port_name}")
            return False

        suc, self.state_right_arm_port = open_buffered_bottle_port(self.state_right_arm_port_name)
        if not suc:
            logerr(f"Failed to open {self.state_right_arm_port_name}")
            return False

        suc, self.state_left_arm_port = open_buffered_bottle_port(self.state_left_arm_port_name)
        if not suc:
            logerr(f"Failed to open {self.state_left_arm_port_name}")
            return False

        return True

    def connect_yarp_ports(self)->bool:
        connection_status = yarp.NetworkBase_connect(self.gaze_cmd_port_name, "/iKinGazeCtrl/rpc", "tcp")
        if not connection_status:
            logerr("gaze control port couldn't connect")

        connection_status = yarp.NetworkBase_connect(self.action_cmd_port_name, "/actionsRenderingEngine/cmd:io", "tcp")
        if not connection_status:
            logerr("action control port couldn't connect")
            return False

        # ctp service ports
        connection_status = yarp.NetworkBase_connect(self.ctp_torso_client_port_name, "/ctpservice/torso/rpc", "tcp")

        if not connection_status:
            logerr("ctp torso control port couldn't connect")
            return False

        connection_status = yarp.NetworkBase_connect(self.ctp_right_arm_client_port_name, "/ctpservice/right_arm/rpc", "tcp")
        if not connection_status:
            logerr("ctp right_arm control port couldn't connect")
            return False

        connection_status = yarp.NetworkBase_connect( self.ctp_left_arm_client_port_name,"/ctpservice/left_arm/rpc", "tcp")
        if not connection_status:
            logerr("ctp left_arm control port couldn't connect")
            return False

        # status ports
        connection_status = yarp.NetworkBase_connect("/"+self.robot_name_yarp+"/torso/state:o", self.state_torso_port_name, "tcp")
        if not connection_status:
            logerr("state torso port couldn't connect")
            return False

        connection_status = yarp.NetworkBase_connect("/"+self.robot_name_yarp+"/right_arm/state:o",self.state_right_arm_port_name,  "tcp")
        if not connection_status:
            logerr("state right_arm port couldn't connect")
            return False

        connection_status = yarp.NetworkBase_connect("/"+self.robot_name_yarp+"/left_arm/state:o",self.state_left_arm_port_name , "tcp")
        if not connection_status:
            logerr("state left_arm port couldn't connect")
            return False

        return True


    def exit(self):
        if self.initialized:
            self.disconnect_and_remove()


    def disconnect_and_remove(self):
        loginfo("interrupting yarp ports")
        self.gaze_client_port.interrupt()
        self.action_client_port.interrupt()
        self.ctp_torso_client_port.interrupt()
        self.ctp_right_arm_client_port.interrupt()
        self.ctp_left_arm_client_port.interrupt()
        self.state_torso_port.interrupt()
        self.state_right_arm_port.interrupt()
        self.state_left_arm_port.interrupt()

        loginfo("closing yarp ports")
        self.gaze_client_port.close()
        self.action_client_port.close()
        self.ctp_torso_client_port.close()
        self.ctp_right_arm_client_port.close()
        self.ctp_left_arm_client_port.close()
        self.state_torso_port.close()
        self.state_right_arm_port.close()
        self.state_left_arm_port.close()


