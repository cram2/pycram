from typing_extensions import List, Union, Optional
from numpy.linalg import norm
from numpy import array
from geometry_msgs.msg import Vector3

from owlready2 import Thing

from ....robot_plans import PickUpAction, PickUpAction
from ....datastructures.pose import PoseStamped, TransformStamped
from ....datastructures.enums import Arms, Grasp
from ....robot_description import RobotDescription, KinematicChainDescription
from ....designator import ObjectDesignatorDescription
from ....logging import loginfo


class DualArmPickupAction(PickUpAction):
    """
    Specialization version of the PickUpAction designator which uses heuristics to solve for a dual pickup solution.
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 grasps: List[Grasp], resolver=None):
        """
        Specialized version of the PickUpAction designator which uses heuristics to solve for a dual pickup problem. The
        designator will choose the arm which is closest to the object that is to be picked up.

        :param object_designator_description: List of object designator which should be picked up
        :param grasps: List of possible grasps which should be used for the pickup
        :param resolver: Optional specialized_designators that returns a performable designator with elements from the
                         lists of possible parameter
        """
        super().__init__(object_designator_description,
                         arms=[Arms.LEFT, Arms.RIGHT],
                         grasps=grasps,
                         resolver=resolver)

        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description

        left_gripper = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT)
        right_gripper = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT)
        self.gripper_list: List[KinematicChainDescription] = [left_gripper, right_gripper]


    def ground(self) -> PickUpAction:
        if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object):
            obj_desig = self.object_designator_description
        else:
            obj_desig = self.object_designator_description.resolve()

        loginfo("Calculating closest gripper to object {}".format(obj_desig.name))

        local_transformer = LocalTransformer()

        object_pose: PoseStamped = obj_desig.world_object.pose
        distances = []
        # Iterate over possible grippers
        for gripper in self.gripper_list:
            # Object pose in gripper frame
            gripper_frame = World.robot.get_link_tf_frame(gripper.get_tool_frame())

            object_T_gripper: PoseStamped = local_transformer.transform_pose(object_pose, gripper_frame)
            object_V_gripper: Vector3 = object_T_gripper.pose.position  # translation vector
            distance = norm(array([object_V_gripper.x, object_V_gripper.y, object_V_gripper.z]))
            loginfo(f"Distance between {gripper} and {obj_desig.name}: {distance}")
            distances.append(distance)

        min_index = distances.index(min(distances))
        winner = self.gripper_list[min_index]
        loginfo(f"Winner is {winner.arm_type.name} with distance {min(distances):.2f}")

        return PickUpAction(object_designator=obj_desig, arm=winner.arm_type, grasp=self.grasps[0])
