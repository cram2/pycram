from . import BodyPart
from .base import PhysicalObject


class AgentPart(BodyPart):
    """
    An agent part is a part of an agent's body.
    """


class HumanPart(BodyPart):
    """
    A human part is a part of a human's body.
    """


class RobotPart(AgentPart):
    """
    A robot part is a part of a robot's body, could be a link or a set of links or a joint, etc.
    """


class Agent(PhysicalObject):
    """
    An agent is an entity that acts.
    """
    has_part = [AgentPart]


class Robot(Agent):
    """
    A robot is a machine that can carry out a complex series of actions automatically.
    """
    has_part = [RobotPart]


class Human(Agent):
    """
    A human is a physical and biological agent
    """
    has_part = [HumanPart]


class PrehensileEffector(AgentPart):
    """
    A prehensile effector is a part of an agent's body that is used for grasping.
    """


class Hand(PrehensileEffector):
    """
    A hand is a prehensile effector that is used for grasping, it has a palm, fingers, and a thumb.
    """


class HumanHand(HumanPart, Hand):
    """
    A human hand is a prehensile that is used for grasping.
    """


class EndEffector(RobotPart, PrehensileEffector):
    """
    An end effector is a device or tool that is connected to the end of a robot arm,
     that is designed to interact with and manipulate the environment.
    """


class Gripper(EndEffector):
    """
    A gripper is a device that can grasp and hold objects.
    """
