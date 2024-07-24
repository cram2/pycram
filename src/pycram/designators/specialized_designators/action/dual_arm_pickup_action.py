import rospy
from typing_extensions import List, Union, Optional
from numpy.linalg import norm
from numpy import array
from geometry_msgs.msg import Vector3

from owlready2 import Thing

from pycram.designators.action_designator import PickUpAction, PickUpActionPerformable
from pycram.local_transformer import LocalTransformer
from pycram.datastructures.world import World
from pycram.datastructures.pose import Transform
from pycram.datastructures.enums import Arms, Grasp
from pycram.robot_description import RobotDescription
from pycram.designator import ObjectDesignatorDescription


class DualArmPickupAction(PickUpAction):
    """
    Specialization version of the PickUpAction designator which uses heuristics to solve for a dual pickup solution.
    """

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 grasps: List[Grasp], resolver=None,
                 ontology_concept_holders: Optional[List[Thing]] = None):
        """
        Specialized version of the PickUpAction designator which uses heuristics to solve for a dual pickup problem. The
        designator will choose the arm which is closest to the object that is to be picked up.
        :param object_designator_description: List of object designator which should be picked up
        :param grasps: List of possible grasps which should be used for the pickup
        :param resolver: Optional specialized_designators that returns a performable designator with elements from the
                         lists of possible parameter
        :param ontology_concept_holders: List of ontology concepts that the action is categorized as or associated with
        """
        super().__init__(object_designator_description,
                         arms=[Arms.LEFT, Arms.RIGHT],
                         grasps=grasps,
                         resolver=resolver,
                         ontology_concept_holders=ontology_concept_holders)

        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description

        left_gripper_frame = World.robot.get_link_tf_frame(
            RobotDescription.current_robot_description.kinematic_chains.get('left').get_tool_frame())
        right_gripper_frame = World.robot.get_link_tf_frame(
            RobotDescription.current_robot_description.kinematic_chains.get('right').get_tool_frame())
        self.gripper_list: List[str] = [left_gripper_frame, right_gripper_frame]


    def ground(self) -> PickUpActionPerformable:
        if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object):
            obj_desig = self.object_designator_description
        else:
            obj_desig = self.object_designator_description.resolve()

        rospy.loginfo("Calculating closest gripper to object {}".format(obj_desig.name))

        local_transformer = LocalTransformer()

        object_frame = obj_desig.world_object.tf_frame
        distances = []
        # Iterate over possible grippers
        for gripper in self.gripper_list:
            transform: Transform = local_transformer.lookup_transform_from_source_to_target_frame(gripper,
                                                                                                  object_frame)
            translation: Vector3 = transform.translation
            distance = norm(array([translation.x, translation.y, translation.z]))
            rospy.loginfo("Distance between {} and {}: {}".format(gripper, obj_desig.name, str(distance)))
            distances.append(distance)

        min_index = distances.index(min(distances))
        winner_frame = self.gripper_list[min_index]

        if winner_frame == World.robot.get_link_tf_frame(
                RobotDescription.current_robot_description.kinematic_chains.get('left').get_tool_frame()):
            winner = Arms.LEFT
            rospy.loginfo("Returning left arm as the winner")
        elif winner_frame == World.robot.get_link_tf_frame(
                RobotDescription.current_robot_description.kinematic_chains.get('right').get_tool_frame()):
            winner = Arms.RIGHT
            rospy.loginfo("Returning right arm as the winner")
        else:
            raise ValueError(f"Could not determine the winner arm for the pickup action. Winner: {winner_frame}")

        return PickUpActionPerformable(object_designator=obj_desig, arm=winner, grasp=self.grasps[0])
