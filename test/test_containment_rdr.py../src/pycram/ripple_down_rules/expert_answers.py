from pycram.datastructures.world_entity import PhysicalBody
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.world import World
from typing import List


def _get_value_9535ab77c3474f5293617f401716d266(case):
    def pose_stamped_contained_objects_of_type_physical_body(case: PoseStamped) -> List[PhysicalBody]:
        """Get possible value(s) for PoseStamped.contained_objects  of type PhysicalBody."""
        # Write your code here
        result = []
        for obj in World.current_world.objects:
            for link in obj.links.values():
                if link.get_axis_aligned_bounding_box().contains(*case.position.to_list()):
                    result.append(link)
        return result
    return pose_stamped_contained_objects_of_type_physical_body(case)
    



'===New Answer==='


from pycram.datastructures.world_entity import PhysicalBody
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.world import World
from typing import List


def _get_value_1261bb6e02e9462ea798f0bd272c6b04(case):
    def pose_stamped_contained_objects_of_type_physical_body(case: PoseStamped) -> List[PhysicalBody]:
        """Get possible value(s) for PoseStamped.contained_objects  of type PhysicalBody."""
        # Write your code here
        result = []
        for obj in World.current_world.objects:
            for link in obj.links.values():
                if link.get_axis_aligned_bounding_box().contains(*case.position.to_list()):
                    result.append(link)
        return result
    return pose_stamped_contained_objects_of_type_physical_body(case)
    



'===New Answer==='


from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.world import World


def _get_value_2fcf6d8dc1f34a6b9e0ca31ce1edeadd(case):
    def conditions_for_pose_stamped_contained_objects_of_type_physical_body(case: PoseStamped) -> bool:
        """Get conditions on whether it's possible to conclude a value for PoseStamped.contained_objects  of type PhysicalBody."""
        return len(World.current_world.objects) >= 2
    return conditions_for_pose_stamped_contained_objects_of_type_physical_body(case)
    



'===New Answer==='


