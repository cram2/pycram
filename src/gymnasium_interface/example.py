from Pycram_gym_env import PyCRAMGymEnv
from pycram.datastructures.enums import Arms, Grasp
from pycram.datastructures.pose import Pose

def custom_reward(state):
    """
    Custom reward function.

    :param state: The current state of the environment.
    :type state: dict
    :return: Reward value based on the state.
    :rtype: float
    """
    return 10 if state else -1

def get_navigate_params(target_pose):
    """
    Generate parameters for the 'navigate' action.

    :param target_pose: The target pose to navigate to.
    :type target_pose: Pose
    :return: Parameters for navigation.
    :rtype: dict
    """
    return {"target_pose": target_pose}

def get_pick_up_params(object_desig, arm, grasps):
    """
    Generate parameters for the 'pick_up' action.

    :param object_desig: Designation of the object to pick up.
    :type object_desig: str
    :param arm: The arm to use for the action.
    :type arm: Arms
    :param grasps: The list of grasps to use.
    :type grasps: list[Grasp]
    :return: Parameters for picking up an object.
    :rtype: dict
    """
    return {"object_desig": object_desig, "arm": arm, "grasps": grasps}

# Define actions and their parameter generators
actions = ["navigate", "pick_up"]
action_param_generators = {
    "navigate": lambda: get_navigate_params(Pose([1.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0])),
    "pick_up": lambda: get_pick_up_params("milk", Arms.RIGHT, [Grasp.FRONT]),
}

# Define objects to initialize in the environment
objects = [
    {"name": "milk", "type": "object", "urdf": "milk.stl", "pose": Pose([2.5, 2.10, 1.02])},
]

# Initialize the environment
env = PyCRAMGymEnv(
    actions,
    {action: generator() for action, generator in action_param_generators.items()},
    objects=objects,
    reward_function=custom_reward
)

# Example usage
state = env.reset()
print("State after reset:", state)
state, reward, done, truncated, info = env.step(1, action_param_generators["pick_up"]())
print("State after step:", state, "Reward:", reward)
