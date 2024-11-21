from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import ObjectType, WorldMode, Grasp
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction, PickUpAction, PlaceAction, OpenAction, CloseAction
from pycram.designators.object_designator import BelieveObject
from pycram.process_module import simulated_robot

class PyCRAMTaskExecutor:
    def __init__(self):
        """Initializes task executor for PyCRAM actions."""
        self.world = BulletWorld(WorldMode.GUI)
        self.robot = None
        self.apartment = None

    def clear_world(self):
        """Removes all objects from the BulletWorld."""
        print("Clearing all objects from BulletWorld...")
        for obj in list(self.world.objects):
            obj.remove()
        print("All objects removed from BulletWorld.")

    def reset_task(self, objects):
        """Resets the simulation environment dynamically."""
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

    def execute_action(self, action, params):
        """Executes a PyCRAM action."""
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

    def _navigate(self, params):
        target_pose = params.get("target_pose")
        if not target_pose:
            raise ValueError("Missing parameter: target_pose")
        NavigateAction(target_locations=[target_pose]).resolve().perform()

    def _pick_up(self, params):
        object_name = params.get("object_desig")
        arm = params.get("arm")
        grasps = params.get("grasps", [Grasp.RIGHT])
        if not object_name or not arm:
            raise ValueError("Missing parameters: object_desig and arm are required")
        object_desig = BelieveObject(names=[object_name])
        action = PickUpAction(
            object_designator_description=object_desig, arms=[arm], grasps=grasps
        ).resolve()
        action.perform()

    def _place(self, params):
        object_desig = params.get("object_desig")
        target_pose = params.get("target_pose")
        arm = params.get("arm")
        if not object_desig or not target_pose or not arm:
            raise ValueError("Missing parameters: object_desig, target_pose, and arm are required")
        PlaceAction(object_designator_description=object_desig, target_locations=[target_pose], arms=[arm]).resolve().perform()

    def _open(self, params):
        handle_desig = params.get("handle_desig")
        arm = params.get("arm")
        if not handle_desig or not arm:
            raise ValueError("Missing parameters: handle_desig and arm are required")
        OpenAction(handle_desig, [arm]).resolve().perform()

    def _close(self, params):
        handle_desig = params.get("handle_desig")
        arm = params.get("arm")
        if not handle_desig or not arm:
            raise ValueError("Missing parameters: handle_desig and arm are required")
        CloseAction(handle_desig, [arm]).resolve().perform()

    def get_current_state(self):
        """Fetches the current state of the environment."""
        robot_pose = self.robot.get_pose() if self.robot else None
        objects = [{"name": obj.name, "pose": obj.pose} for obj in self.world.objects]
        return {"robot_pose": robot_pose, "objects": objects}
