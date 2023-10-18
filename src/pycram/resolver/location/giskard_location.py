from pycram.external_interfaces.giskard import achieve_cartesian_goal
from pycram.designators.location_designator import CostmapLocation
from pycram.bullet_world import Use_shadow_world, BulletWorld
from pycram.helper import _apply_ik
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.pose_generator_and_validator import reachability_validator
from typing import Tuple, Dict

import tf
import numpy as np


class GiskardLocation(CostmapLocation):

    def __iter__(self) -> CostmapLocation.Location:
        """
        Resolves a CostmapLocation for reachability to a specific Location using Giskard. Since Giskard is able to perform
        full body IK solving we can use this to get the Pose of a robot at which it is able to reach a certain point.
        This resolver only supports reachable_for and not visible_for

        :param desig: A CostmapLocation Designator description
        :return: An instance of CostmapLocation.Location with a pose from which the robot can reach the target
        """
        if self.reachable_for:
            pose_right, end_config_right = self._get_reachable_pose_for_arm(self.target,
                                                                            robot_description.get_tool_frame("right"))
            pose_left, end_config_left = self._get_reachable_pose_for_arm(self.target,
                                                                          robot_description.get_tool_frame("left"))

            test_robot = BulletWorld.current_bullet_world.get_shadow_object(BulletWorld.robot)
            with Use_shadow_world():
                valid, arms = reachability_validator(pose_right, test_robot, self.target, {})
                if valid:
                    yield CostmapLocation.Location(pose_right, arms)
                valid, arms = reachability_validator(pose_left, test_robot, self.target, {})
                if valid:
                    yield self.Location(pose_left, arms)

    def _get_reachable_pose_for_arm(self, target: Pose, end_effector_link: str) -> Tuple[Pose, Dict]:
        """
        Calls Giskard to perform full body ik solving between the map and the given end effector link. The end joint
        configuration of the robot as well as its end pose are then returned.

        :param target: The pose which the robots end effector should reach
        :param end_effector_link: The name of the end effector which should reach the target
        :return: The end pose of the robot as well as its final joint configuration
        """
        giskard_result = achieve_cartesian_goal(target, end_effector_link, "map")
        joints = giskard_result.trajectory.joint_names
        trajectory_points = giskard_result.trajectory.points

        end_config = dict(zip(joints, trajectory_points[-1].positions))
        orientation = list(tf.transformations.quaternion_from_euler(0, 0, end_config["yaw"], axes="sxyz"))
        pose = Pose([end_config["x"], end_config["y"], 0], orientation)
        return pose, end_config
