from enum import Enum

from pycram.datastructures.enums import JointType
from ..multiverse_extras.exceptions import UnsupportedJointType


class MultiverseAPIName(Enum):
    """
    Enum for the different APIs of the Multiverse.
    """
    GET_CONTACT_BODIES = "get_contact_bodies"
    GET_CONSTRAINT_EFFORT = "get_constraint_effort"
    ATTACH = "attach"
    DETACH = "detach"
    GET_RAYS = "get_rays"
    EXIST = "exist"
    PAUSE = "pause"
    UNPAUSE = "unpause"
    SAVE = "save"
    LOAD = "load"


class MultiverseProperty(Enum):
    def __str__(self):
        return self.value


class MultiverseBodyProperty(MultiverseProperty):
    """
    Enum for the different properties of a body the Multiverse.
    """
    POSITION = "position"
    ORIENTATION = "quaternion"
    RELATIVE_VELOCITY = "relative_velocity"


class MultiverseJointProperty(MultiverseProperty):
    pass


class MultiverseJointPosition(MultiverseJointProperty):
    """
    Enum for the Position names of the different joint types in the Multiverse.
    """
    REVOLUTE_JOINT_POSITION = "joint_rvalue"
    PRISMATIC_JOINT_POSITION = "joint_tvalue"

    @classmethod
    def from_pycram_joint_type(cls, joint_type: 'JointType') -> 'MultiverseJointPosition':
        if joint_type in [JointType.REVOLUTE, JointType.CONTINUOUS]:
            return MultiverseJointPosition.REVOLUTE_JOINT_POSITION
        elif joint_type == JointType.PRISMATIC:
            return MultiverseJointPosition.PRISMATIC_JOINT_POSITION
        else:
            raise UnsupportedJointType(joint_type)


class MultiverseJointCMD(MultiverseJointProperty):
    """
    Enum for the Command names of the different joint types in the Multiverse.
    """
    REVOLUTE_JOINT_CMD = "cmd_joint_rvalue"
    PRISMATIC_JOINT_CMD = "cmd_joint_tvalue"

    @classmethod
    def from_pycram_joint_type(cls, joint_type: 'JointType') -> 'MultiverseJointCMD':
        if joint_type in [JointType.REVOLUTE, JointType.CONTINUOUS]:
            return MultiverseJointCMD.REVOLUTE_JOINT_CMD
        elif joint_type == JointType.PRISMATIC:
            return MultiverseJointCMD.PRISMATIC_JOINT_CMD
        else:
            raise UnsupportedJointType(joint_type)
