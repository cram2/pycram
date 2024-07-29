from abc import ABC
from enum import Enum


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
    """
    Enum for the different properties of a joint the Multiverse.
    """
    REVOLUTE_JOINT_POSITION = "joint_rvalue"
    PRISMATIC_JOINT_POSITION = "joint_tvalue"
    REVOLUTE_JOINT_CMD = "cmd_joint_rvalue"
    PRISMATIC_JOINT_CMD = "cmd_joint_tvalue"

    def is_cmd(self):
        return self in [MultiverseJointProperty.REVOLUTE_JOINT_CMD,
                        MultiverseJointProperty.PRISMATIC_JOINT_CMD]
