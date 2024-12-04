from pycram.language import Language
from pycram.world_concepts.world_object import Object, World
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
import gymnasium as gym

class PyCRAMGym(gym.Env):
    """
    First small implementation of the PyCRAMGYM, this class can either be used to store necessary information for an experiment or let
    a robot explore an environment, with a set of actions. The exploration will be done via reinforcement learning.
    """

    world: str # should be World currently not possible because spawning multiple bullet worlds leads to problems
    robot: str # name to search for robot
    plan: Language # plan language object that can performed


    def __init__(self, inWorld: str, inRobot: str, inPlan: Language):
        super().__init__()
        if World.current_world is not None:
            if inRobot.endswith('.urdf'):
                self.robot = Object(inRobot.split('.urdf')[0], ObjectType.ROBOT, inRobot, pose=Pose([1, 2, 0]))
            else:
                self.robot = Object(inRobot, ObjectType.ROBOT, inRobot+".urdf", pose=Pose([1, 2, 0]))
            self.world = Object(inWorld, ObjectType.ENVIRONMENT, inWorld + ".urdf")
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
