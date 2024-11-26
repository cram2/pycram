import logging
from gymnasium_interface.pycram_gym_env import PyCRAMGymEnv
from pycram.datastructures.enums import Arms, Grasp
from pycram.datastructures.pose import Pose

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

def custom_reward(state):
    """
    Custom reward function.

    :param state: The current state of the environment.
    :type state: dict
    :return: Reward value based on the state.
    :rtype: float
    """
    return 10.0 if state else -1.0

# Define actions as a list of strings
actions = ["navigate", "pick_up"]

# Define default parameters for each action
default_params = {
    "navigate": {"target_pose": Pose(position=[1.0, 2.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])},
    "pick_up": {"object_desig": "milk", "arm": Arms.RIGHT, "grasps": [Grasp.FRONT]},
}

# Define objects to initialize in the environment
objects = [
    {
        "name": "milk",
        "type": "object",
        "urdf": "milk.stl",
        "pose": Pose(position=[2.5, 2.10, 1.02]),
    }
]

# Initialize the Gymnasium environment
env = PyCRAMGymEnv(actions=actions, default_params=default_params, objects=objects, reward_function=custom_reward)

# Reset the environment and retrieve the initial state
state, info = env.reset()
logging.info(f"State after reset: {state}")

# Perform a step in the environment
try:
    state, reward, done, truncated, info = env.step(
        action=1,  # Index of the action to execute
        params={"object_desig": "milk", "arm": Arms.RIGHT, "grasps": [Grasp.FRONT]},
    )
    logging.info(f"State after step: {state}, Reward: {reward}, Done: {done}, Truncated: {truncated}")
except ValueError as e:
    logging.error(f"Action failed: {e}")
