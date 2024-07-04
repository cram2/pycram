from typing_extensions import List, Union, Optional
from numpy.linalg import norm
from numpy import array
from geometry_msgs.msg import Vector3

from owlready2 import Thing

from pycram.designators.action_designator import PickUpAction, PickUpActionPerformable
from pycram.local_transformer import LocalTransformer
from pycram.datastructures.world import World
from pycram.datastructures.pose import Transform
from pycram.robot_descriptions import robot_description
from pycram.designator import ObjectDesignatorDescription


class DualArmPickupAction(PickUpAction):
    """
    Specialization version of the PickUpAction designator which uses heuristics to solve for a dual pickup solution.
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 grasps: List[str], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Specialized version of the PickUpAction designator which uses heuristics to solve for a dual pickup solution.
        :param object_designator_description: List of object designator which should be picked up
        :param grasps: List of possible grasps which should be used for the pickup
        :param resolver: Optional specialized_designators that returns a performable designator with elements from the
                         lists of possible parameter
        :param ontology_concept_holders: List of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(object_designator_description,
                         arms=['left', 'right'],
                         grasps=grasps,
                         resolver=resolver,
                         ontology_concept_holders=ontology_concept_holders)

        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description

        self.local_transformer = LocalTransformer()
        self.robot = World.robot
        left_gripper_frame = self.robot.get_link_tf_frame(robot_description.get_tool_frame('left'))
        right_gripper_frame = self.robot.get_link_tf_frame(robot_description.get_tool_frame('right'))
        self.gripper_list: List[str] = [left_gripper_frame, right_gripper_frame]
        self.grasps: List[str] = grasps


    def ground(self) -> PickUpActionPerformable:
        if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object):
            obj_desig = self.object_designator_description
        else:
            obj_desig = self.object_designator_description.resolve()

        object_frame = obj_desig.world_object.tf_frame
        distances = []
        # Iterate over possible grippers
        for gripper in self.gripper_list:
            transform: Transform = self.local_transformer.lookup_transform_from_source_to_target_frame(object_frame, gripper)
            translation: Vector3 = transform.translation
            distance = norm(array([translation.x, translation.y, translation.z]))
            distances.append(distance)

        min_index = distances.index(min(distances))
        winner_frame = self.gripper_list[min_index]

        if "l_gripper" in winner_frame:
            winner = "left"
        elif "r_gripper" in winner_frame:
            winner = "right"
        else:
            raise ValueError(f"Could not determine the winner arm for the pickup action. Winner: {winner_frame}")

        return PickUpActionPerformable(object_designator=obj_desig, arm=winner, grasp=self.grasps[0])
