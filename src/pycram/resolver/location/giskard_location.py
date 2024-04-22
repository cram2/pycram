import rospy

from ...designators.location_designator import CostmapLocation
from ...external_interfaces.ik import request_giskard_ik
from ...robot_description import ManipulatorDescription
from ...world import UseProspectionWorld, World
from ...robot_descriptions import robot_description
from ...local_transformer import LocalTransformer
from ...plan_failures import IKError


class GiskardLocation(CostmapLocation):
    """'
    Specialization version of the CostmapLocation which uses Giskard to solve for a full-body IK solution. This
    designator is especially useful for robots which lack a degree of freedom and therefore need to use the base to
    manipulate the environment effectively.
    """

    def __iter__(self) -> CostmapLocation.Location:
        """
        Uses Giskard to perform full body ik solving to get the pose of a robot at which it is able to reach a certain point.

        :yield: An instance of CostmapLocation.Location with a pose from which the robot can reach the target
        """
        local_transformer = LocalTransformer()
        target_map = local_transformer.transform_pose(self.target, "map")

        manipulator_descs = list(
            filter(lambda chain: isinstance(chain[1], ManipulatorDescription), robot_description.chains.items()))
        for name, chain in manipulator_descs:
            try:
                pose, joint_states = request_giskard_ik(target_map, World.robot, chain.tool_frame)
                yield self.Location(pose, chain.name)
            except IKError as e:
                pass

