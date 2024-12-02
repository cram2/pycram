from .base import PhysicalObject

class Agent(PhysicalObject):
    """
    An agent is an entity that acts.
    """

class Robot(Agent):
    """
    A robot is a machine that can carry out a complex series of actions automatically.
    """

class Human(Agent):
    ...