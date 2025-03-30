from ..language import Language
from ..world_concepts.world_object import Object, World
from ..datastructures.enums import ObjectType
from ..datastructures.pose import PoseStamped
import gymnasium as gym
import pycrap

class PyCRAMGym(gym.Env):
    """
    First small implementation of the PyCRAMGYM, this class can either be used to store necessary information for an experiment or let
    a robot explore an environment, with a set of actions. The exploration will be done via reinforcement learning.
    """

    world: str # should be World currently not possible because spawning multiple bullet worlds leads to problems
    """
    The world should be an real World Object, but sadly this is not possible, because if we close and open multiple 
    bulletworlds in quick succession, then there is a problem with objects not being loaded properly. Thus we decided to
    let the user create the world they want to use and just load the environment urdf of the file they give to this class
    """
    robot: str # name to search for robot
    """
    The robot should be a real Robot Object, but in order to stay in line with the world it is just the name of the 
    urdf that will be loaded as a robot.
    """
    plan: Language # plan language object that can performed
    """
    Plan langauge object that can get executed. 
    """

    def __init__(self, inWorld: str, inRobot: str, inPlan: Language):
        super().__init__()
        if World.current_world is not None:
            if inRobot.endswith('.urdf'):
                self.robot = Object(inRobot.split('.urdf')[0], pycrap.Robot, inRobot, pose=PoseStamped.from_list([1, 2, 0]))
            else:
                self.robot = Object(inRobot, pycrap.Robot, inRobot +".urdf", pose=PoseStamped.from_list([1, 2, 0]))
            self.world = Object(inWorld, pycrap.Apartment, inWorld + ".urdf")
            self.plan = inPlan

    def step(self, action: gym.core.ActType):
        """
        Ticks the world, in our case this means we pick one possible action and perform it.

        :ivar action: The action to take.
        """
        return None

    def reset(self):
        """
        Returns the environment to the normal state.
        """
        self.world.reset()

    def close(self):
        """
        Resets the world to a completely empty state. (Should close world at some point)
        """
        if World.current_world is not None and self.world is not None:
            World.current_world.remove_object(self.world)
            World.current_world.remove_object(self.robot)
