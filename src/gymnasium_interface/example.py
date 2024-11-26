from gymnasium_interface.pycram_gym_env import PyCRAMGymEnv
from pycram.datastructures.enums import Arms, Grasp
from pycram.datastructures.pose import Pose
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def custom_reward(state):
    """
    Custom reward function for the environment.

    :param state: Current state of the environment.
    :type state: dict
    :return: Reward based on the state.
    :rtype: int
    """
    return 10 if state else -1

# Define actions as strings, corresponding to the method names in the TaskExecutor
actions = ["navigate", "pick_up"]

# Default parameters for each action
default_params = {
    "navigate": {"target_pose": Pose([1.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0])},
    "pick_up": {"object_desig": "milk", "arm": Arms.RIGHT, "grasps": [Grasp.FRONT]},
}

# Define objects to initialize in the environment
objects = [
    {
        "name": "milk",
        "type": "object",
        "urdf": "milk.stl",
        "pose": Pose([2.5, 2.10, 1.02])
    }
]

# Initialize the environment
env = PyCRAMGymEnv(actions, default_params, objects=objects, reward_function=custom_reward)

# Reset the environment
state, info = env.reset()
logging.info(f"State after reset: {state}")

# Perform a step in the environment
state, reward, done, truncated, info = env.step(
    action=1,  # Index of the action to execute
    params={"object_desig": "milk", "arm": Arms.RIGHT, "grasps": [Grasp.FRONT]}
)
logging.info(f"State after step: {state}, Reward: {reward}, Done: {done}")
