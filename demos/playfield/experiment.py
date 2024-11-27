from pycram.language import Language
from pycram.world_concepts.world_object import Object, World
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
# from gymnasium import core
import gymnasium as gym

# from gymnasium_interface.pycram_gym_env import PyCRAMGymEnv

class Experiment(gym.Env):
    """
    Class that takes a pointer to the world and a robot name. Will take care of (de)spawning the desired robot
    and running the plan. Plan needs to be proper defined and of the plan language.

    :ivar world: filename of the enviroment that should be loaded
    :ivar robot: Name of the robot that executes the plan
    :ivar plan: pycram.planlanguage object that contains the whole plan if present
    """

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
        Class that takes a pointer to the world and a robot name. Will take care of (de)spawning the desired robot
        and running the plan. Plan needs to be proper defined and of the plan language.

        :ivar world: filename of the enviroment that should be loaded
        :ivar robot: Name of the robot that executes the plan
        :ivar plan: pycram.planlanguage object that contains the whole plan if present
        """
        return None

    def reset(self):
        """
        Class that takes a pointer to the world and a robot name. Will take care of (de)spawning the desired robot
        and running the plan. Plan needs to be proper defined and of the plan language.

        :ivar world: filename of the enviroment that should be loaded
        :ivar robot: Name of the robot that executes the plan
        :ivar plan: pycram.planlanguage object that contains the whole plan if present
        """
        self.world.reset()

    def close(self):
        """
        Class that takes a pointer to the world and a robot name. Will take care of (de)spawning the desired robot
        and running the plan. Plan needs to be proper defined and of the plan language.

        :ivar world: filename of the enviroment that should be loaded
        :ivar robot: Name of the robot that executes the plan
        :ivar plan: pycram.planlanguage object that contains the whole plan if present
        """
        if World.current_world is not None and self.world is not None:
            World.current_world.remove_object(self.world)
            World.current_world.remove_object(self.robot)
