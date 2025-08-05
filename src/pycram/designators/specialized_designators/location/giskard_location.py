from ..tf_transformations import quaternion_from_euler

from ....datastructures.enums import StaticJointState
from ....datastructures.pose import PoseStamped
from ....designators.location_designator import CostmapLocation
from ....external_interfaces.giskard import projection_cartesian_goal_with_approach, projection_joint_goal
from ....multirobot import RobotManager
from ....datastructures.world import UseProspectionWorld, World
from ....local_transformer import LocalTransformer
from ....costmaps import OccupancyCostmap, GaussianCostmap
from ....pose_generator_and_validator import PoseGenerator


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
        if self.visible_for:
            raise ValueError("GiskardLocation does not support the visible_for parameter")
        local_transformer = LocalTransformer()
        target_map = local_transformer.transform_pose(self.target, "map")

        manipulator_descs = RobotManager.get_robot_description().get_manipulator_chains()

        near_costmap = (OccupancyCostmap(0.35, False, 200, 0.02, target_map)
                        + GaussianCostmap(200, 15, 0.02, target_map))
        for maybe_pose in PoseGenerator(near_costmap, 200):
            for chain in manipulator_descs:
                projection_joint_goal(chain.get_static_joint_states(StaticJointState.Park), allow_collisions=True)

                trajectory = projection_cartesian_goal_with_approach(maybe_pose, target_map, chain.tool_frame,
                                                                     "map", RobotManager.get_robot_description().base_link)
                last_point_positions = trajectory.trajectory.points[-1].positions
                last_point_names = trajectory.trajectory.joint_names
                last_joint_states = dict(zip(last_point_names, last_point_positions))
                orientation = list(
                    quaternion_from_euler(0, 0, last_joint_states["brumbrum_yaw"], axes="sxyz"))
                pose = PoseStamped.from_list([last_joint_states["brumbrum_x"], last_joint_states["brumbrum_y"], 0], orientation)

                robot_joint_states = {}
                for joint_name, state in last_joint_states.items():
                    if joint_name in World.robot.joints.keys():
                        robot_joint_states[joint_name] = state

                prospection_robot = World.current_world.get_prospection_object_for_object(World.robot)

                with UseProspectionWorld():
                    prospection_robot.set_multiple_joint_positions(robot_joint_states)
                    prospection_robot.set_pose(pose)
                    gripper_pose = prospection_robot.get_link_pose(chain.get_tool_frame())

                    if gripper_pose.position.euclidean_distance(target_map.position) <= 0.02:
                        yield CostmapLocation.Location(pose, [chain.arm_type])


