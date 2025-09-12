import pathlib
import unittest

from pycram.datastructures.grasp import GraspDescription
from pycram.robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    CameraDescription, RobotDescriptionManager
from pycram.datastructures.enums import Arms, GripperState, StaticJointState, Grasp, ApproachDirection, \
    VerticalAlignment


class TestRobotDescription(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.path = str(pathlib.Path(__file__).parent.resolve()) + '/../resources/robots/' + "pr2" + '.urdf'
        cls.path_turtlebot = str(pathlib.Path(__file__).parent.resolve()) + '/../resources/robots/' + "turtlebot" + '.urdf'
        cls.urdf_obj = URDF(cls.path)

    def test_robot_description_construct(self):
        robot_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        self.assertEqual(robot_description.name, "pr2")
        self.assertEqual(robot_description.base_link, "base_link")
        self.assertEqual(robot_description.torso_link, "torso_lift_link")
        self.assertEqual(robot_description.torso_joint, "torso_lift_joint")
        self.assertTrue(type(robot_description.urdf_object) is URDF)
        self.assertEqual(len(robot_description.links), 88)
        self.assertEqual(len(robot_description.joints), 87)

    def test_kinematic_chain_description_construct(self):
        chain = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link", self.urdf_obj,
                                          arm_type=Arms.LEFT)
        self.assertEqual(chain.name, "left")
        self.assertEqual(chain.start_link, "torso_lift_link")
        self.assertEqual(chain.end_link, "l_wrist_roll_link")
        self.assertEqual(chain.arm_type, Arms.LEFT)
        self.assertTrue("torso_lift_link" in chain.links)
        self.assertTrue("l_wrist_roll_link" in chain.links)
        self.assertEqual(len(chain.links), 10)
        self.assertEqual(len(chain.joints), 7)

    def test_end_effector_description_construct(self):
        end_effector = EndEffectorDescription("left_gripper", "l_gripper_palm_link", "l_gripper_tool_frame",
                                              self.urdf_obj)
        self.assertEqual(end_effector.name, "left_gripper")
        self.assertEqual(end_effector.start_link, "l_gripper_palm_link")
        self.assertEqual(end_effector.tool_frame, "l_gripper_tool_frame")
        self.assertTrue("l_gripper_palm_link" in end_effector.links)
        self.assertEqual(len(end_effector.links), 11)
        self.assertEqual(len(end_effector.joints), 10)

    def test_camera_description_construct(self):
        camera = CameraDescription("kinect_camera", "wide_stereo_optical_frame", 1.27,
                                   1.60, 0.99483, 0.75049)
        self.assertEqual(camera.name, "kinect_camera")
        self.assertEqual(camera.link_name, "wide_stereo_optical_frame")
        self.assertEqual(camera.minimal_height, 1.27)
        self.assertEqual(camera.maximal_height, 1.60)
        self.assertEqual(camera.horizontal_angle, 0.99483)
        self.assertEqual(camera.vertical_angle, 0.75049)
        self.assertEqual(camera.front_facing_axis, [0, 0, 1])

    def test_kinematic_chain_description_add_to_robot_description(self):
        chain = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link", self.urdf_obj,
                                          arm_type=Arms.LEFT)
        robot_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        robot_description.add_kinematic_chain_description(chain)
        self.assertTrue(chain in robot_description.kinematic_chains.values())
        self.assertEqual(robot_description.kinematic_chains["left"], chain)
        self.assertEqual(robot_description.get_arm_chain(Arms.LEFT), chain)
        self.assertEqual(len(robot_description.kinematic_chains), 1)

    def test_end_effector_description_add_to_kinematic_chain_description(self):
        chain = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link", self.urdf_obj,
                                          arm_type=Arms.LEFT)
        end_effector = EndEffectorDescription("left_gripper", "l_gripper_palm_link", "l_gripper_tool_frame",
                                              self.urdf_obj)
        chain.end_effector = end_effector
        self.assertEqual(chain.end_effector, end_effector)
        self.assertEqual(chain.get_tool_frame(), "l_gripper_tool_frame")

    def test_camera_description_add_to_robot_description(self):
        camera = CameraDescription("kinect_camera", "wide_stereo_optical_frame", 1.27,
                                   1.60, 0.99483, 0.75049)
        robot_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        robot_description.add_camera_description(camera)
        self.assertTrue(camera in robot_description.cameras.values())
        self.assertEqual(robot_description.cameras["kinect_camera"], camera)
        self.assertEqual(len(robot_description.cameras), 1)

    def test_kinematic_chain_description_add_static_joint_states(self):
        chain = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link", self.urdf_obj,
                                          arm_type=Arms.LEFT)
        chain.add_static_joint_states(StaticJointState.Park, {'l_shoulder_pan_joint': 1.712,
                                               'l_shoulder_lift_joint': -0.264,
                                               'l_upper_arm_roll_joint': 1.38,
                                               'l_elbow_flex_joint': -2.12,
                                               'l_forearm_roll_joint': 16.996,
                                               'l_wrist_flex_joint': -0.073,
                                               'l_wrist_roll_joint': 0.0})
        self.assertTrue(StaticJointState.Park in chain.static_joint_states)
        self.assertEqual(chain.static_joint_states[StaticJointState.Park], {'l_shoulder_pan_joint': 1.712,
                                                             'l_shoulder_lift_joint': -0.264,
                                                             'l_upper_arm_roll_joint': 1.38,
                                                             'l_elbow_flex_joint': -2.12,
                                                             'l_forearm_roll_joint': 16.996,
                                                             'l_wrist_flex_joint': -0.073,
                                                             'l_wrist_roll_joint': 0.0})

    def test_end_effector_description_add_static_joint_states(self):
        chain = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link", self.urdf_obj,
                                          arm_type=Arms.LEFT)
        end_effector = EndEffectorDescription("left_gripper", "l_gripper_palm_link", "l_gripper_tool_frame",
                                              self.urdf_obj)
        chain.end_effector = end_effector
        end_effector.add_static_joint_states(GripperState.OPEN, {'l_gripper_l_finger_joint': 0.548,
                                                                 'l_gripper_r_finger_joint': 0.548})
        self.assertTrue(GripperState.OPEN in end_effector.static_joint_states.keys())
        self.assertEqual(end_effector.static_joint_states[GripperState.OPEN], {'l_gripper_l_finger_joint': 0.548,
                                                                               'l_gripper_r_finger_joint': 0.548})
        self.assertEqual(chain.get_static_gripper_state(GripperState.OPEN), {'l_gripper_l_finger_joint': 0.548,
                                                                             'l_gripper_r_finger_joint': 0.548})

    def test_get_parent_link(self):
        robot_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        self.assertEqual("base_link", robot_description.get_parent("torso_lift_joint"))
        self.assertEqual("l_wrist_roll_joint", robot_description.get_parent("l_wrist_roll_link"))

    def test_get_parent_joint(self):
        robot_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        self.assertEqual("torso_lift_joint", robot_description.get_parent("torso_lift_link"))
        self.assertEqual("l_wrist_roll_joint", robot_description.get_parent("l_wrist_roll_link"))

    def test_get_child_links(self):
        robot_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        self.assertEqual("l_forearm_link", robot_description.get_child("l_forearm_joint", return_multiple_children=True))
        self.assertEqual("sensor_mount_link", robot_description.get_child("sensor_mount_frame_joint"))

    def test_get_child_joints(self):
        robot_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        self.assertEqual(["projector_wg6802418_frame_joint", "sensor_mount_frame_joint"],
                         robot_description.get_child("head_plate_frame", return_multiple_children=True))
        self.assertEqual("base_bellow_joint", robot_description.get_child("base_link"))

    def test_get_manipulation_chain(self):
        chain_left = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link", self.urdf_obj,
                                          arm_type=Arms.LEFT)
        end_effector = EndEffectorDescription("left_gripper", "l_gripper_palm_link", "l_gripper_tool_frame",
                                              self.urdf_obj)
        chain_right = KinematicChainDescription("right", "torso_lift_link", "r_wrist_roll_link", self.urdf_obj, arm_type=Arms.RIGHT)
        chain_left.end_effector = end_effector
        robot_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        robot_description.add_kinematic_chain_description(chain_left)
        robot_description.add_kinematic_chain_description(chain_right)
        self.assertEqual(chain_left, robot_description.kinematic_chains["left"])
        self.assertEqual(chain_left, robot_description.get_arm_chain(Arms.LEFT))
        self.assertEqual([chain_left], robot_description.get_manipulator_chains())
        self.assertTrue(chain_right not in robot_description.get_manipulator_chains())

    def test_get_arm_chains(self):
        chain_left = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link", self.urdf_obj,
                                          arm_type=Arms.LEFT)
        chain_right = KinematicChainDescription("right", "torso_lift_link", "r_wrist_roll_link", self.urdf_obj, arm_type=Arms.RIGHT)
        robot_description = RobotDescription("pr2", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        robot_description.add_kinematic_chain_description(chain_left)
        robot_description.add_kinematic_chain_description(chain_right)
        self.assertEqual(chain_left, robot_description.get_arm_chain(Arms.LEFT))
        self.assertEqual([chain_left, chain_right], robot_description.get_arm_chain(Arms.BOTH))

    def test_get_tool_frame(self):
        chain_left = KinematicChainDescription("left", "torso_lift_link", "l_wrist_roll_link", self.urdf_obj,
                                          arm_type=Arms.LEFT)
        end_effector = EndEffectorDescription("left_gripper", "l_gripper_palm_link", "l_gripper_tool_frame",
                                                self.urdf_obj)
        chain_left.end_effector = end_effector
        self.assertEqual("l_gripper_tool_frame", chain_left.get_tool_frame())

    def test_robot_description_manager_singelton(self):
        rdm = RobotDescriptionManager()
        rdm2 = RobotDescriptionManager()
        self.assertEqual(rdm, rdm2)
        self.assertIs(rdm, rdm2)

    def test_register_robot_description(self):
        robot_description = RobotDescription("pr2_test1", "base_link", "torso_lift_link", "torso_lift_joint", self.path)
        rdm = RobotDescriptionManager()
        rdm.register_description(robot_description)
        self.assertEqual(rdm.descriptions["pr2_test1"], robot_description)
        self.assertTrue(robot_description in rdm.descriptions.values())

    def test_load_robot_description(self):
        rdm = RobotDescriptionManager()
        rdm.load_description(self.urdf_obj.name) # loads description by the name of the urdf
        self.assertIs(RobotDescription.current_robot_description, rdm.descriptions["pr2"])

    def test_robot_description_turtlebot(self):
        robot_description = RobotDescription("turtlebot", "base_link", "base_link", "base_joint", self.path_turtlebot)
        self.assertEqual(robot_description.name, "turtlebot")
        self.assertEqual(robot_description.base_link, "base_link")
        self.assertEqual(robot_description.torso_link, "base_link")
        self.assertEqual(robot_description.torso_joint, "base_joint")
        self.assertTrue(type(robot_description.urdf_object) is URDF)
        self.assertEqual(len(robot_description.links), 11)
        self.assertEqual(len(robot_description.joints), 10)

    def test_grasp_descriptions(self):
        grasp1 = GraspDescription(ApproachDirection.LEFT, VerticalAlignment.TOP, True)
        grasp2 = GraspDescription(ApproachDirection.BACK, VerticalAlignment.BOTTOM, False)
        grasp3 = GraspDescription(ApproachDirection.RIGHT, VerticalAlignment.NoAlignment, True)

        grasp1_quat = [0.7071067811865476, 0.0, -0.7071067811865476, 0.0]
        grasp2_quat = [0.7071067811865476, 0.0, 0.7071067811865476, 0.0]
        grasp3_quat = [0.5, 0.5, 0.5, 0.5]

        end_effector = EndEffectorDescription("left_gripper", "l_gripper_palm_link", "l_gripper_tool_frame",
                                              self.urdf_obj)

        end_effector.update_all_grasp_orientations([0, 0, 0, 1])

        self.assertEqual(grasp1_quat, end_effector.grasps[grasp1])
        self.assertEqual(grasp2_quat, end_effector.grasps[grasp2])
        self.assertEqual(grasp3_quat, end_effector.grasps[grasp3])

