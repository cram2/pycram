import numpy as np
import rospy
from threading import Lock
from typing import Any

from ..datastructures.enums import JointType, PerceptionTechniques
from ..external_interfaces.tmc import tmc_gripper_control, tmc_talk
from ..robot_description import RobotDescription
from ..process_module import ProcessModule
from ..datastructures.pose import Point
from ..robot_descriptions import robot_description
from ..utils import _apply_ik
from ..external_interfaces.ik import request_ik
from .. import world_reasoning as btr
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..external_interfaces import giskard
from ..world_concepts.world_object import Object
from ..datastructures.world import World
from pydub import AudioSegment
from pydub.playback import play
from gtts import gTTS
import io


###########################################################
########## Process Modules for the Real HSRB ###############
###########################################################


class HSRBNavigationReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
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
        # todo at the moment perception ignores searching for a specific object type so we do as well on real
        # if desig.technique == 'human' and (desig.state == "start" or desig.state == None):
        #     human_pose = queryHuman()
        #     pose = Pose.from_pose_stamped(human_pose)
        #     pose.position.z = 0
        #     human = []
        #     human.append(Object("human", ObjectType.HUMAN, "human_male.stl", pose=pose))
        #     object_dict = {}
        #
        #     # Iterate over the list of objects and store each one in the dictionary
        #     for i, obj in enumerate(human):
        #         object_dict[obj.name] = obj
        #     return object_dict
        #
        #     return human_pose
        # elif desig.technique == 'human' and desig.state == "stop":
        #     stop_queryHuman()
        #     return "stopped"
        #
        # query_result = queryEmpty(ObjectDesignatorDescription(types=[desig.object_type]))
        # perceived_objects = []
        # for i in range(0, len(query_result.res)):
        #     # this has to be pose from pose stamped since we spawn the object with given header
        #     obj_pose = Pose.from_pose_stamped(query_result.res[i].pose[0])
        #     # obj_pose.orientation = [0, 0, 0, 1]
        #     # obj_pose_tmp = query_result.res[i].pose[0]
        #     obj_type = query_result.res[i].type
        #     obj_size = query_result.res[i].shape_size
        #     obj_color = query_result.res[i].color[0]
        #     color_switch = {
        #         "red": [1, 0, 0, 1],
        #         "green": [0, 1, 0, 1],
        #         "blue": [0, 0, 1, 1],
        #         "black": [0, 0, 0, 1],
        #         "white": [1, 1, 1, 1],
        #         # add more colors if needed
        #     }
        #     color = color_switch.get(obj_color)
        #     if color is None:
        #         color = [0, 0, 0, 1]
        #
        #     # atm this is the string size that describes the object but it is not the shape size thats why string
        #     def extract_xyz_values(input_string):
        #         # Split the input string by commas and colon to separate key-value pairs
        #         # key_value_pairs = input_string.split(', ')
        #
        #         # Initialize variables to store the X, Y, and Z values
        #         x_value = None
        #         y_value = None
        #         z_value = None
        #
        #         for key in input_string:
        #             x_value = key.dimensions.x
        #             y_value = key.dimensions.y
        #             z_value = key.dimensions.z
        #
        #         #
        #         # # Iterate through the key-value pairs to extract the values
        #         # for pair in key_value_pairs:
        #         #     key, value = pair.split(': ')
        #         #     if key == 'x':
        #         #         x_value = float(value)
        #         #     elif key == 'y':
        #         #         y_value = float(value)
        #         #     elif key == 'z':
        #         #         z_value = float(value)
        #
        #         return x_value, y_value, z_value
        #
        #     x, y, z = extract_xyz_values(obj_size)
        #     size = (x, z / 2, y)
        #     size_box = (x / 2, z / 2, y / 2)
        #     hard_size = (0.02, 0.02, 0.03)
        #     id = World.current_world.add_rigid_box(obj_pose, hard_size, color)
        #     box_object = Object(obj_type + "_" + str(rospy.get_time()), obj_type, pose=obj_pose, color=color, id=id,
        #                         customGeom={"size": [hard_size[0], hard_size[1], hard_size[2]]})
        #     box_object.set_pose(obj_pose)
        #     box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.type, box_object)
        #
        #     perceived_objects.append(box_desig)
        #
        # object_dict = {}
        #
        # # Iterate over the list of objects and store each one in the dictionary
        # for i, obj in enumerate(perceived_objects):
        #     object_dict[obj.name] = obj
        # return object_dict


class HSRBMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real HSRB while avoiding all collisions
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
    Moves the arm joints of the real HSRB to the given configuration while avoiding all collisions
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
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class HSRBCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class HSRBTalkReal(ProcessModule):
    """
    Tries to close an already grasped container
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
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.teleport_robot(designator.target)


class HSRBTalkSemiReal(ProcessModule):
    """
    Tries to close an already grasped container
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
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBNavigationReal(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBNavigationSemiReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveHeadReal(self._looking_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBDetectingReal(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveTCPReal(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveJointsReal(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveGripperReal(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBOpenReal(self._open_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBCloseReal(self._close_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBCloseReal(self._close_lock)

    def talk(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBTalkReal(self._talk_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBTalkSemiReal(self._talk_lock)
