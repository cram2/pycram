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
