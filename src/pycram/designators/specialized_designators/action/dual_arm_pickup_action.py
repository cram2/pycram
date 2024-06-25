from pycram.designators.action_designator import PickUpAction, PickUpActionPerformable
from pycram.local_transformer import LocalTransformer
from pycram.datastructures.world import World
from pycram.robot_descriptions import robot_description


class DualArmPickupAction(PickUpAction):
    """
    Specialization version of the PickUpAction designator which uses heuristics to solve for a dual grasping solution.
    """

    def __init__(self, *args, **kwargs):
        self.local_transformer = LocalTransformer()
        self.robot = World.robot
        left_gripper_frame = self.robot.get_link_tf_frame(robot_description.get_tool_frame('left'))
        right_gripper_frame = self.robot.get_link_tf_frame(robot_description.get_tool_frame('right'))
        self.gripper_list = [left_gripper_frame, right_gripper_frame]

        super().__init__(*args, **kwargs)

    def ground(self) -> PickUpActionPerformable:

        pass
