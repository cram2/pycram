import numpy as np
from threading import Lock
from typing_extensions import Any

from ..datastructures.enums import ExecutionType
from ..external_interfaces.tmc import tmc_gripper_control, tmc_talk
from ..robot_description import RobotDescription
from ..process_module import ProcessModule
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..external_interfaces import giskard
from ..datastructures.world import World
from pydub import AudioSegment
from pydub.playback import play
from gtts import gTTS

import io

from ..ros.logging import logdebug


class HSRBNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = World.robot
        robot.set_pose(desig.target)


class HSRBDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """
   # pass
    def _execute(self, desig: DetectingMotion) -> Any:
        pass


###########################################################
########## Process Modules for the Real HSRB ###############
###########################################################


class HSRBNavigationReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        logdebug(f"Sending goal to giskard to Move the robot")
        # giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")
        # todome fix this
        # queryPoseNav(designator.target)


class HSRBMoveHeadReal(ProcessModule):
    """
    Process module for the real HSRB that sends a pose goal to giskard to move the robot head
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        giskard.move_head_to_pose(target)


class HSRBDetectingReal(ProcessModule):
    """
    Process Module for the real HSRB that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, desig: DetectingMotion) -> Any:
        pass


class HSRBMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real HSRB while avoiding all collisions via giskard
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
        giskard.avoid_all_collisions()
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(
            designator.arm).get_tool_frame(), "map")


class HSRBMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real HSRB to the given configuration while avoiding all collisions via giskard
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class HSRBMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class HSRBMoveGripperReal(ProcessModule):
    """
     Opens or closes the gripper of the real HSRB with the help of giskard.
     """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        tmc_gripper_control(designator)


class HSRBOpenReal(ProcessModule):
    """
    This process Modules tries to open an already grasped container via giskard
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class HSRBCloseReal(ProcessModule):
    """
    This process module executes close a an already grasped container via giskard
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class HSRBTalkReal(ProcessModule):
    """
    Let the robot speak over tmc interface.
    """

    def _execute(self, designator: TalkingMotion) -> Any:
        tmc_talk(designator)


###########################################################
########## Process Modules for the Semi Real HSRB ###############
###########################################################
class HSRBNavigationSemiReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        logdebug(f"Sending goal to giskard to Move the robot")
        giskard.teleport_robot(designator.target)


class HSRBTalkSemiReal(ProcessModule):
    """
    Low Level implementation to let the robot talk using gTTS and pydub.
    """

    def _execute(self, designator: TalkingMotion) -> Any:
        """
        Convert text to speech using gTTS, modify the pitch and play it without saving to disk.
        """
        sentence = designator.cmd
        # Create a gTTS object
        tts = gTTS(text=sentence, lang='en', slow=False)

        # Save the speech to an in-memory file
        mp3_fp = io.BytesIO()
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)

        # Load the audio into pydub from the in-memory file
        audio = AudioSegment.from_file(mp3_fp, format="mp3")

        # Speed up the audio slightly
        faster_audio = audio.speedup(playback_speed=1.2)

        # Play the modified audio
        play(faster_audio)


###########################################################
########## HSRB MANAGER ###############
###########################################################
class HSRBManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("hsrb")
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()
        self._talk_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBNavigationReal(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBNavigationSemiReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveHeadReal(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return HSRBDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBDetectingReal(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBDetecting(self._detecting_lock)

    def move_tcp(self):
        if  ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveTCPReal(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveJointsReal(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBMoveGripperReal(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBOpenReal(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBCloseReal(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBCloseReal(self._close_lock)

    def talk(self):
        if ProcessModuleManager.execution_type == ExecutionType.REAL:
            return HSRBTalkReal(self._talk_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.SEMI_REAL:
            return HSRBTalkSemiReal(self._talk_lock)
