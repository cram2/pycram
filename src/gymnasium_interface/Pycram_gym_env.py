import gymnasium as gym
from gymnasium.spaces import Discrete
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import ObjectType, WorldMode, Grasp
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction, PickUpAction, PlaceAction, OpenAction, CloseAction
from pycram.designators.object_designator import BelieveObject
from pycram.process_module import simulated_robot


class PyCRAMTaskExecutor:
    """
    Handles task execution in a PyCRAM environment. This class integrates with BulletWorld for
    managing objects and robot tasks in the simulation.

    Attributes:
        world (BulletWorld): The BulletWorld instance managing the environment.
        robot (Object): The robot object in the environment.
        apartment (Object): The apartment or environment object in the simulation.
    """

    def __init__(self):
        self.world = BulletWorld(WorldMode.GUI)
        self.robot = None
        self.apartment = None

    def clear_world(self) -> None:
        """Removes all objects from the BulletWorld."""
        print("Clearing all objects from BulletWorld...")
        for obj in list(self.world.objects):
            obj.remove()
        print("All objects removed from BulletWorld.")

    def reset_task(self, objects: list[dict]) -> None:
        """
        Resets the simulation environment dynamically.

        Args:
            objects (list[dict]): List of objects to be added to the environment.
        """
        self.clear_world()

        # Reload the apartment URDF
        self.apartment = Object("apartment", "environment", "apartment.urdf")

        # Reinitialize the robot
        self.robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1.2, 1, 0]))
        self.world.robot = self.robot

        # Add dynamic objects
        for obj in objects:
            name = obj["name"]
            obj_type = obj["type"]
            urdf = obj["urdf"]
            pose = obj["pose"]

            print(f"Adding object: {name}, URDF path: {urdf}, Pose: {pose}")

            existing_object = self.world.get_object_by_name(name)
            if existing_object:
                print(f"Reusing existing object: {name}")
            else:
                Object(name, obj_type, urdf, pose=pose)

        print("Environment reset: Apartment, robot, and dynamic objects added.")

    def execute_action(self, action: str, params: dict) -> None:
        """
        Executes a PyCRAM action.

        Args:
            action (str): The action to be executed (e.g., "navigate", "pick_up").
            params (dict): Parameters required for the action.
        """
        with simulated_robot:
            if action == "navigate":
                self._navigate(params)
            elif action == "pick_up":
                self._pick_up(params)
            elif action == "place":
                self._place(params)
            elif action == "open":
                self._open(params)
            elif action == "close":
                self._close(params)
            else:
                raise ValueError(f"Unknown action: {action}")

    def get_current_state(self) -> dict:
        """
        Fetches the current state of the environment.

        Returns:
            dict: Current state containing robot pose and objects' states.
        """
        robot_pose = self.robot.get_pose() if self.robot else None
        objects = [{"name": obj.name, "pose": obj.pose} for obj in self.world.objects]
        return {"robot_pose": robot_pose, "objects": objects}


class PyCRAMGymEnv(gym.Env):
    def __init__(self, actions: list, default_params: dict = None, objects: list = None, reward_function: callable = None):
        """
        Initializes the Gymnasium environment.

        Args:
            actions (list): List of valid action names (e.g., ["navigate", "pick_up", "place"]).
            default_params (dict, optional): Default parameters for actions if not provided by the user.
            objects (list, optional): List of objects to add to the environment.
            reward_function (callable, optional): User-defined function to calculate rewards.
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
        """
        Resets the environment.

        Returns:
            tuple: The initial state of the environment.
        """
        with simulated_robot:
            self.executor.reset_task(self.objects)
            self.state = self.executor.get_current_state()
        return self.state, {}

    def step(self, action: int, params: dict = None):
        """
        Executes a step in the environment.

        Args:
            action (int): The action index to execute.
            params (dict, optional): Parameters for the action.

        Returns:
            tuple: A tuple containing the state, reward, done flag, and additional info.
        """
        with simulated_robot:
            action_name = self.actions[action]
            action_params = self.default_params.get(action_name, {}).copy()
            if params:
                action_params.update(params)

            # Execute the action using designators directly
            if action_name == "navigate":
                target_pose = action_params.get("target_pose")
                if not target_pose:
                    raise ValueError("Missing parameter: target_pose")
                NavigateAction(target_locations=[target_pose]).resolve().perform()

            elif action_name == "pick_up":
                object_name = action_params.get("object_desig")
                arm = action_params.get("arm")
                grasps = action_params.get("grasps", [Grasp.RIGHT])
                if not object_name or not arm:
                    raise ValueError("Missing parameters: object_desig and arm are required")
                object_desig = BelieveObject(names=[object_name])
                PickUpAction(
                    object_designator_description=object_desig, arms=[arm], grasps=grasps
                ).resolve().perform()

            elif action_name == "place":
                object_desig = action_params.get("object_desig")
                target_pose = action_params.get("target_pose")
                arm = action_params.get("arm")
                if not object_desig or not target_pose or not arm:
                    raise ValueError("Missing parameters: object_desig, target_pose, and arm are required")
                PlaceAction(
                    object_designator_description=object_desig, target_locations=[target_pose], arms=[arm]
                ).resolve().perform()

            elif action_name == "open":
                handle_desig = action_params.get("handle_desig")
                arm = action_params.get("arm")
                if not handle_desig or not arm:
                    raise ValueError("Missing parameters: handle_desig and arm are required")
                OpenAction(handle_desig, [arm]).resolve().perform()

            elif action_name == "close":
                handle_desig = action_params.get("handle_desig")
                arm = action_params.get("arm")
                if not handle_desig or not arm:
                    raise ValueError("Missing parameters: handle_desig and arm are required")
                CloseAction(handle_desig, [arm]).resolve().perform()

            else:
                raise ValueError(f"Unknown action: {action_name}")

            # Update the state
            self.state = self.executor.get_current_state()

            # Calculate reward
            reward = self._calculate_reward()

            # Placeholder: done logic can be updated later
            done = self._is_done()

        return self.state, reward, done, False, {}

    def _calculate_reward(self):
        """Calculates the reward using the user-defined reward function."""
        if self.reward_function:
            return self.reward_function(self.state)
        return 1.0

    def _is_done(self):
        """Checks if the task is complete."""
        return False
