import pinocchio
import numpy as np

from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.pinocchio_ik import create_joint_configuration, \
    parse_configuration_vector_to_joint_positions, inverse_kinematics_logarithmic, clip_joints_to_limits, compute_ik
from pycram.failures import IKError
from pycram.testing import BulletWorldTestCase


class PinocchioTest(BulletWorldTestCase):

    def test_create_configuration(self):
        self.world.reset_current_world()
        model = pinocchio.buildModelFromUrdf(self.robot.path)
        config = np.array([1., 0., 1., 0., 1., 0., 1., 0., 1., 0., 1., 0., 1., 0., 1., 0., 1.,
                            0., 1., 0., 1., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.,
                            1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
                            0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.])
        self.assertTrue(np.array_equal(create_joint_configuration(self.robot, model), config))
        self.assertEqual(len(create_joint_configuration(self.robot, model)), 64)

    def test_parse_config_to_joint_dict(self):
        model = pinocchio.buildModelFromUrdf(self.robot.path)
        zero_config = pinocchio.neutral(model)
        joint_dict = parse_configuration_vector_to_joint_positions(zero_config, model)
        self.assertEqual(sum(list(joint_dict.values())), 0)
        self.assertEqual(list(self.robot.joints.keys()), list(self.robot.joints.keys()))

    def test_changed_joint_configuration(self):
        self.robot.set_joint_position("torso_lift_joint", 0.3)
        model = pinocchio.buildModelFromUrdf(self.robot.path)
        config = create_joint_configuration(self.robot, model)
        self.assertTrue(0.3 in config)

        joint_config = parse_configuration_vector_to_joint_positions(config, model)
        self.assertEqual(joint_config["torso_lift_joint"], 0.3)

    def test_clip_values(self):
        values = [10, -10, 10]
        lower = [3, 2, 6]
        upper = [8, 5, 8]

        self.assertTrue(np.array_equal(clip_joints_to_limits(values, lower, upper), np.array([8, 2, 8])))


    def test_ik_computation(self):
        target_link = "r_gripper_tool_frame"
        target_pose = PoseSteamped.from_list([0.5, 0.2, 0.6])
        result_configuration = compute_ik(target_link, target_pose, self.robot)

        self.assertTrue(np.any(result_configuration))

    def test_ik_computation_error(self):
        target_link = "r_gripper_tool_frame"
        target_pose = PoseSteamped.from_list([5, 0.2, 0.6])
        self.assertRaises(IKError, compute_ik, target_link, target_pose, self.robot)