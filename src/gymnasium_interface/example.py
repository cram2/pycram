from gymnasium_interface.Pycram_gym_env import PyCRAMGymEnv
from pycram.datastructures.enums import Arms, Grasp
from pycram.datastructures.pose import Pose

# Example usage
def custom_reward(state):
    return 10 if state else -1

actions = ["navigate", "pick_up"]
default_params = {
    "navigate": {"target_pose": Pose([1.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0])},
    "pick_up": {"object_desig": "milk", "arm": Arms.RIGHT, "grasps": [Grasp.FRONT]},
}
objects = [{"name": "milk", "type": "object", "urdf": "milk.stl", "pose": Pose([2.5, 2.10, 1.02])}]

env = PyCRAMGymEnv(actions, default_params, objects=objects, reward_function=custom_reward)
state = env.reset()
print("State after reset:", state)
state, reward, done, truncated, info = env.step(1, {"object_desig": "milk", "arm": Arms.RIGHT, "grasps": [Grasp.FRONT]})
print("State after step:", state, "Reward:", reward)

