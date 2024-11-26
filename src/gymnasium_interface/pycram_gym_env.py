import gymnasium as gym
from gymnasium.spaces import Discrete
from gymnasium_interface.task_executor import PyCRAMTaskExecutor  # Use absolute import
from pycram.process_module import simulated_robot


class PyCRAMGymEnv(gym.Env):
    """
    A Gymnasium-compatible environment for integrating PyCRAM task execution.

    This environment allows users to execute PyCRAM tasks within a Gymnasium-compatible
    framework. It supports dynamic task initialization, state tracking, and custom reward
    calculations.

    :param actions: List of valid action classes or functions (e.g., [NavigateAction, PickUpAction]).
    :type actions: list
    :param default_params: Default parameters for each action, keyed by action class/function (optional).
    :type default_params: dict
    :param objects: List of objects to initialize in the environment (optional).
    :type objects: list
    :param reward_function: Custom user-defined function to compute rewards (optional).
    :type reward_function: callable
    """

    def __init__(self, actions, default_params=None, objects=None, reward_function=None):
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
        """
        Resets the environment.

        :return: The initial state of the environment.
        :rtype: tuple
        """
        with simulated_robot:
            self.executor.reset_task(self.objects)
            self.state = self.executor.get_current_state()
        return self.state, {}

    def step(self, action, params=None):
        """
        Executes a step in the environment.

        :param action: The action index to execute.
        :type action: int
        :param params: Additional parameters for the action.
        :type params: dict, optional
        :return: A tuple containing the next state, reward, done flag, truncated flag, and additional info.
        :rtype: tuple
        """
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
        """
        Fetches the current state of the environment.

        :return: The current state of the environment.
        :rtype: dict
        """
        return self.state

    def _calculate_reward(self):
        """
        Calculates the reward using the user-defined reward function.

        :return: The calculated reward.
        :rtype: float
        """
        if self.reward_function:
            return self.reward_function(self.state)
        return 1.0

    def _is_done(self):
        """
        Checks if the task is complete.

        :return: True if the task is done, otherwise False.
        :rtype: bool
        """
        return False
