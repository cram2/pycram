import gymnasium as gym
from gymnasium.spaces import Discrete
from .task_executor import PyCRAMTaskExecutor
from pycram.process_module import simulated_robot
import sys

class PyCRAMGymEnv(gym.Env):
    def __init__(self, actions, default_params=None, objects=None, reward_function=None):
        """
        Initializes the Gymnasium environment.

        Args:
            actions (list): List of valid action names (e.g., ["navigate", "pick_up", "place"]).
            default_params (dict): Default parameters for actions if not provided by the user.
            objects (list): List of objects to add to the environment.
            reward_function (function): User-defined function to calculate rewards.
        """
        super().__init__()
        self.actions = actions
        self.default_params = default_params or {}
        self.objects = objects or []
        self.reward_function = reward_function

        # Dynamically define the action space
        self.action_space = Discrete(len(actions))

        # Initialize the task executor
        self.executor = PyCRAMTaskExecutor()

        # Initialize the state
        self.state = None
        self.reset()

    def reset(self):
        """Resets the environment."""
        with simulated_robot:
            self.executor.reset_task(self.objects)
            self.state = self.executor.get_current_state()
        return self.state, {}

    def step(self, action, params=None):
        """Executes a step in the environment."""
        with simulated_robot:
            action_name = self.actions[action]
            action_params = self.default_params.get(action_name, {}).copy()
            if params:
                action_params.update(params)

            # Execute the action
            self.executor.execute_action(action_name, action_params)

            # Update the state
            self.state = self._get_observation()

            # Calculate reward
            reward = self._calculate_reward()

            # Placeholder: done logic can be updated later
            done = self._is_done()

        return self.state, reward, done, False, {}

    def _get_observation(self):
        """Fetches the current state of the environment."""
        return self.state

    def _calculate_reward(self):
        """Calculates the reward using the user-defined reward function."""
        if self.reward_function:
            return self.reward_function(self.state)
        return 1.0

    def _is_done(self):
        """Checks if the task is complete."""
        return False
