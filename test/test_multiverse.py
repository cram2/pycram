#!/usr/bin/env python3
import os
import unittest

import numpy as np
import psutil
from tf.transformations import quaternion_from_euler, quaternion_multiply
from typing_extensions import Optional, List

from pycram.datastructures.dataclasses import ContactPointsList, ContactPoint, AxisAlignedBoundingBox
from pycram.datastructures.enums import ObjectType, Arms, JointType
from pycram.datastructures.pose import Pose
from pycram.robot_description import RobotDescriptionManager
from pycram.world_concepts.world_object import Object
from pycram.validation.error_checkers import calculate_angle_between_quaternions
from pycram.helper import get_robot_mjcf_path, parse_mjcf_actuators

multiverse_installed = True
try:
    from pycram.worlds.multiverse import Multiverse
except ImportError:
    multiverse_installed = False

processes = psutil.process_iter()
process_names = [p.name() for p in processes]
multiverse_running = True
mujoco_running = True
if 'multiverse_server' not in process_names:
    multiverse_running = False
if 'mujoco' not in process_names:
    mujoco_running = False


@unittest.skipIf(not multiverse_installed, "Multiverse is not installed.")
@unittest.skipIf(not multiverse_running, "Multiverse server is not running.")
@unittest.skipIf(not mujoco_running, "Mujoco is not running.")
class MultiversePyCRAMTestCase(unittest.TestCase):
    if multiverse_installed:
        multiverse: Multiverse
    big_bowl: Optional[Object] = None

    @classmethod
    def setUpClass(cls):
        if not multiverse_installed:
            return
        cls.multiverse = Multiverse()

    @classmethod
    def tearDownClass(cls):
        cls.multiverse.exit(remove_saved_states=True)
        cls.multiverse.remove_multiverse_resources()

    def tearDown(self):
        self.multiverse.remove_all_objects()

    def test_get_axis_aligned_bounding_box_for_one_link_object(self):
        position = [1, 1, 0.1]
        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1, 1, 0.1],
                                                                     quaternion_from_euler(np.pi/4, 0, 0).tolist()))
        aabb = milk.get_axis_aligned_bounding_box()
        self.assertIsInstance(aabb, AxisAlignedBoundingBox)
        min_p_1, max_p_1 = aabb.get_min_max()
        width = max_p_1[0] - min_p_1[0]
        length = max_p_1[1] - min_p_1[1]
        height = max_p_1[2] - min_p_1[2]
        self.assertTrue(width > 0)
        self.assertTrue(length > 0)
        self.assertTrue(height > 0)
        # Move the object and check if the bounding box is updated correctly
        position_shift = 1
        milk.set_position([position[0] + position_shift, position[1] + position_shift, 0.1])
        aabb = milk.get_axis_aligned_bounding_box()
        min_p_2, max_p_2 = aabb.get_min_max()
        width_2 = max_p_2[0] - min_p_2[0]
        length_2 = max_p_2[1] - min_p_2[1]
        height_2 = max_p_2[2] - min_p_2[2]
        self.assertAlmostEqual(width, width_2, delta=0.001)
        self.assertAlmostEqual(length, length_2, delta=0.001)
        self.assertAlmostEqual(height, height_2, delta=0.001)
        for i in range(3):
            self.assertAlmostEqual(min_p_1[0] + position_shift, min_p_2[0], delta=0.001)
            self.assertAlmostEqual(max_p_1[0] + position_shift, max_p_2[0], delta=0.001)

    def test_spawn_xml_object(self):
        bread = Object("bread_1", ObjectType.GENERIC_OBJECT, "bread_1.xml", pose=Pose([1, 1, 0.1]))
        self.assert_poses_are_equal(bread.get_pose(), Pose([1, 1, 0.1]))

    def test_spawn_mesh_object(self):
        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1, 1, 0.1]))
        self.assert_poses_are_equal(milk.get_pose(), Pose([1, 1, 0.1]))
        self.multiverse.simulate(0.2)
        contact_points = milk.contact_points()
        self.assertTrue(len(contact_points) > 0)

    def test_parse_mjcf_actuators(self):
        mjcf_file = get_robot_mjcf_path("pal_robotics", "tiago_dual")
        self.assertTrue(os.path.exists(mjcf_file))
        joint_actuators = parse_mjcf_actuators(mjcf_file)
        self.assertIsInstance(joint_actuators, dict)
        self.assertTrue(len(joint_actuators) > 0)
        self.assertTrue("arm_left_1_joint" in joint_actuators)
        self.assertTrue("arm_right_1_joint" in joint_actuators)
        self.assertTrue(joint_actuators["arm_right_1_joint"] == "arm_right_1_actuator")

    def test_get_actuator_for_joint(self):
        robot = self.spawn_robot()
        joint_name = "arm_right_1_joint"
        actuator_name = robot.get_actuator_for_joint(robot.joints[joint_name])
        self.assertEqual(actuator_name, "arm_right_1_actuator")

    def test_get_images_for_target(self):
        robot = self.spawn_robot(robot_name='pr2')
        camera_description = self.multiverse.robot_description.get_default_camera()
        camera_link_name = camera_description.link_name
        camera_pose = robot.get_link_pose(camera_link_name)
        camera_frame = self.multiverse.robot_description.get_camera_frame()
        camera_front_facing_axis = camera_description.front_facing_axis
        milk_spawn_position = np.array(camera_front_facing_axis) * 0.5
        orientation = camera_pose.to_transform(camera_frame).invert().rotation_as_list()
        milk = self.spawn_milk(milk_spawn_position.tolist(), orientation, frame=camera_frame)
        _, depth, segmentation_mask = self.multiverse.get_images_for_target(milk.pose, camera_pose, plot=False)
        self.assertIsInstance(depth, np.ndarray)
        self.assertIsInstance(segmentation_mask, np.ndarray)
        self.assertTrue(depth.shape == (256, 256))
        self.assertTrue(segmentation_mask.shape == (256, 256))
        self.assertTrue(milk.id in np.unique(segmentation_mask).flatten().tolist())
        avg_depth_of_milk = np.mean(depth[segmentation_mask == milk.id])
        self.assertAlmostEqual(avg_depth_of_milk, 0.5, delta=0.1)

    def test_reset_world(self):
        set_position = [1, 1, 0.1]
        milk = self.spawn_milk(set_position)
        milk.set_position(set_position)
        milk_position = milk.get_position_as_list()
        self.assert_list_is_equal(milk_position[:2], set_position[:2], delta=self.multiverse.conf.position_tolerance)
        self.multiverse.reset_world()
        milk_pose = milk.get_pose()
        self.assert_list_is_equal(milk_pose.position_as_list()[:2],
                                  milk.original_pose.position_as_list()[:2],
                                  delta=self.multiverse.conf.position_tolerance)
        self.assert_orientation_is_equal(milk_pose.orientation_as_list(), milk.original_pose.orientation_as_list())

    def test_spawn_robot_with_actuators_directly_from_multiverse(self):
        if self.multiverse.conf.use_controller:
            robot_name = "tiago_dual"
            rdm = RobotDescriptionManager()
            rdm.load_description(robot_name)
            self.multiverse.spawn_robot_with_controller(robot_name, Pose([-2, -2, 0.001]))

    def test_spawn_object(self):
        milk = self.spawn_milk([1, 1, 0.1])
        self.assertIsInstance(milk, Object)
        milk_pose = milk.get_pose()
        self.assert_list_is_equal(milk_pose.position_as_list()[:2], [1, 1],
                                  delta=self.multiverse.conf.position_tolerance)
        self.assert_orientation_is_equal(milk_pose.orientation_as_list(), milk.original_pose.orientation_as_list())

    def test_remove_object(self):
        milk = self.spawn_milk([1, 1, 0.1])
        milk.remove()
        self.assertTrue(milk not in self.multiverse.objects)
        self.assertFalse(self.multiverse.check_object_exists(milk))

    def test_check_object_exists(self):
        milk = self.spawn_milk([1, 1, 0.1])
        self.assertTrue(self.multiverse.check_object_exists(milk))

    def test_set_position(self):
        milk = self.spawn_milk([1, 1, 0.1])
        original_milk_position = milk.get_position_as_list()
        original_milk_position[0] += 1
        milk.set_position(original_milk_position)
        milk_position = milk.get_position_as_list()
        self.assert_list_is_equal(milk_position[:2], original_milk_position[:2],
                                  delta=self.multiverse.conf.position_tolerance)

    def test_update_position(self):
        milk = self.spawn_milk([1, 1, 0.1])
        milk.update_pose()
        milk_position = milk.get_position_as_list()
        self.assert_list_is_equal(milk_position[:2], [1, 1], delta=self.multiverse.conf.position_tolerance)

    def test_set_joint_position(self):
        if self.multiverse.robot is None:
            robot = self.spawn_robot()
        else:
            robot = self.multiverse.robot
        step = 0.2
        for joint in ['torso_lift_joint']:
            joint_type = robot.joints[joint].type
            original_joint_position = robot.get_joint_position(joint)
            robot.set_joint_position(joint, original_joint_position + step)
            joint_position = robot.get_joint_position(joint)
            if not self.multiverse.conf.use_controller:
                delta = self.multiverse.conf.prismatic_joint_position_tolerance if joint_type == JointType.PRISMATIC \
                    else self.multiverse.conf.revolute_joint_position_tolerance
            else:
                delta = 0.18
            self.assertAlmostEqual(joint_position, original_joint_position + step, delta=delta)

    def test_spawn_robot(self):
        if self.multiverse.robot is not None:
            robot = self.multiverse.robot
        else:
            robot = self.spawn_robot(robot_name="pr2")
        self.assertIsInstance(robot, Object)
        self.assertTrue(robot in self.multiverse.objects)
        self.assertTrue(self.multiverse.robot.name == robot.name)

    def test_destroy_robot(self):
        if self.multiverse.robot is None:
            self.spawn_robot()
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)
        self.multiverse.robot.remove()
        self.assertTrue(self.multiverse.robot not in self.multiverse.objects)

    def test_respawn_robot(self):
        self.spawn_robot()
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)
        self.multiverse.robot.remove()
        self.assertTrue(self.multiverse.robot not in self.multiverse.objects)
        self.spawn_robot()
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)

    def test_set_robot_position(self):
        step = -1
        for i in range(3):
            self.spawn_robot()
            new_position = [-3 + step * i, -3 + step * i, 0.001]
            self.multiverse.robot.set_position(new_position)
            robot_position = self.multiverse.robot.get_position_as_list()
            self.assert_list_is_equal(robot_position[:2], new_position[:2],
                                      delta=self.multiverse.conf.position_tolerance)
            self.tearDown()

    def test_set_robot_orientation(self):
        self.spawn_robot()
        for i in range(3):
            current_quaternion = self.multiverse.robot.get_orientation_as_list()
            # rotate by 45 degrees without using euler angles
            rotation_quaternion = quaternion_from_euler(0, 0, np.pi / 4)
            new_quaternion = quaternion_multiply(current_quaternion, rotation_quaternion)
            self.multiverse.robot.set_orientation(new_quaternion)
            robot_orientation = self.multiverse.robot.get_orientation_as_list()
            quaternion_difference = calculate_angle_between_quaternions(new_quaternion, robot_orientation)
            self.assertAlmostEqual(quaternion_difference, 0, delta=self.multiverse.conf.orientation_tolerance)

    def test_set_robot_pose(self):
        self.spawn_robot(orientation=quaternion_from_euler(0, 0, np.pi / 4))
        position_step = -1
        angle_step = np.pi / 4
        num_steps = 10
        self.step_robot_pose(self.multiverse.robot, position_step, angle_step, num_steps)
        position_step = 1
        angle_step = -np.pi / 4
        self.step_robot_pose(self.multiverse.robot, position_step, angle_step, num_steps)

    def step_robot_pose(self, robot, position_step, angle_step, num_steps):
        original_position = robot.get_position_as_list()
        original_orientation = robot.get_orientation_as_list()
        for i in range(num_steps):
            new_position = [original_position[0] + position_step * (i + 1),
                            original_position[1] + position_step * (i + 1), original_position[2]]
            rotation_quaternion = quaternion_from_euler(0, 0, angle_step * (i + 1))
            new_quaternion = quaternion_multiply(original_orientation, rotation_quaternion)
            new_pose = Pose(new_position, new_quaternion)
            self.multiverse.robot.set_pose(new_pose)
            robot_pose = self.multiverse.robot.get_pose()
            self.assert_poses_are_equal(new_pose, robot_pose,
                                        position_delta=self.multiverse.conf.position_tolerance,
                                        orientation_delta=self.multiverse.conf.orientation_tolerance)

    def test_get_environment_pose(self):
        apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment.urdf")
        pose = apartment.get_pose()
        self.assertIsInstance(pose, Pose)

    def test_attach_object(self):
        for _ in range(3):
            milk = self.spawn_milk([1, 0.1, 0.1])
            cup = self.spawn_cup([1, 1.1, 0.1])
            milk.attach(cup)
            self.assertTrue(cup in milk.attachments)
            milk_position = milk.get_position_as_list()
            milk_position[0] += 1
            cup_position = cup.get_position_as_list()
            estimated_cup_position = cup_position.copy()
            estimated_cup_position[0] += 1
            milk.set_position(milk_position)
            new_cup_position = cup.get_position_as_list()
            self.assert_list_is_equal(new_cup_position[:2], estimated_cup_position[:2],
                                      self.multiverse.conf.position_tolerance)
            self.tearDown()

    def test_detach_object(self):
        for i in range(2):
            milk = self.spawn_milk([1, 0, 0.1])
            cup = self.spawn_cup([1, 1, 0.1])
            milk.attach(cup)
            self.assertTrue(cup in milk.attachments)
            milk.detach(cup)
            self.assertTrue(cup not in milk.attachments)
            milk_position = milk.get_position_as_list()
            milk_position[0] += 1
            cup_position = cup.get_position_as_list()
            estimated_cup_position = cup_position.copy()
            milk.set_position(milk_position)
            new_milk_position = milk.get_position_as_list()
            new_cup_position = cup.get_position_as_list()
            self.assert_list_is_equal(new_milk_position[:2], milk_position[:2],
                                      self.multiverse.conf.position_tolerance)
            self.assert_list_is_equal(new_cup_position[:2], estimated_cup_position[:2],
                                      self.multiverse.conf.position_tolerance)
            self.tearDown()

    def test_attach_with_robot(self):
        milk = self.spawn_milk([-1, -1, 0.1])
        robot = self.spawn_robot()
        ee_link = self.multiverse.get_arm_tool_frame_link(Arms.RIGHT)
        # Get position of milk relative to robot end effector
        robot.attach(milk, ee_link.name, coincide_the_objects=False)
        self.assertTrue(robot in milk.attachments)
        milk_initial_pose = milk.root_link.get_pose_wrt_link(ee_link)
        robot_position = 1.57
        robot.set_joint_position("arm_right_2_joint", robot_position)
        milk_pose = milk.root_link.get_pose_wrt_link(ee_link)
        self.assert_poses_are_equal(milk_initial_pose, milk_pose)

    def test_get_object_contact_points(self):
        for i in range(10):
            milk = self.spawn_milk([1, 1, 0.01], [0, -0.707, 0, 0.707])
            contact_points = self.multiverse.get_object_contact_points(milk)
            self.assertIsInstance(contact_points, ContactPointsList)
            self.assertEqual(len(contact_points), 1)
            self.assertIsInstance(contact_points[0], ContactPoint)
            self.assertTrue(contact_points[0].link_b.object, self.multiverse.floor)
            cup = self.spawn_cup([1, 1, 0.2])
            # This is needed because the cup is spawned in the air, so it needs to fall
            # to get in contact with the milk
            self.multiverse.simulate(0.3)
            contact_points = self.multiverse.get_object_contact_points(cup)
            self.assertIsInstance(contact_points, ContactPointsList)
            self.assertEqual(len(contact_points), 1)
            self.assertIsInstance(contact_points[0], ContactPoint)
            self.assertTrue(contact_points[0].link_b.object, milk)
            self.tearDown()

    def test_get_contact_points_between_two_objects(self):
        for i in range(3):
            milk = self.spawn_milk([1, 1, 0.01], [0, -0.707, 0, 0.707])
            cup = self.spawn_cup([1, 1, 0.2])
            # This is needed because the cup is spawned in the air so it needs to fall
            # to get in contact with the milk
            self.multiverse.simulate(0.3)
            contact_points = self.multiverse.get_contact_points_between_two_objects(milk, cup)
            self.assertIsInstance(contact_points, ContactPointsList)
            self.assertEqual(len(contact_points), 1)
            self.assertIsInstance(contact_points[0], ContactPoint)
            self.assertTrue(contact_points[0].link_a.object, milk)
            self.assertTrue(contact_points[0].link_b.object, cup)
            self.tearDown()

    def test_get_one_ray(self):
        milk = self.spawn_milk([1, 1, 0.1])
        intersected_object = self.multiverse.ray_test([1, 2, 0.1], [1, 1.5, 0.1])
        self.assertTrue(intersected_object is None)
        intersected_object = self.multiverse.ray_test([1, 2, 0.1], [1, 1, 0.1])
        self.assertTrue(intersected_object == milk.id)

    def test_get_rays(self):
        milk = self.spawn_milk([1, 1, 0.1])
        intersected_objects = self.multiverse.ray_test_batch([[1, 2, 0.1], [1, 2, 0.1]],
                                                             [[1, 1.5, 0.1], [1, 1, 0.1]])
        self.assertTrue(intersected_objects[0][0] == -1)
        self.assertTrue(intersected_objects[1][0] == milk.id)

    @staticmethod
    def spawn_big_bowl() -> Object:
        big_bowl = Object("big_bowl", ObjectType.GENERIC_OBJECT, "BigBowl.obj",
                          pose=Pose([2, 2, 0.1], [0, 0, 0, 1]))
        return big_bowl

    @staticmethod
    def spawn_milk(position: List, orientation: Optional[List] = None, frame="map") -> Object:
        if orientation is None:
            orientation = [0, 0, 0, 1]
        milk = Object("milk_box", ObjectType.MILK, "milk_box.xml",
                      pose=Pose(position, orientation, frame=frame))
        return milk

    def spawn_robot(self, position: Optional[List[float]] = None,
                    orientation: Optional[List[float]] = None,
                    robot_name: Optional[str] = 'tiago_dual',
                    replace: Optional[bool] = True) -> Object:
        if position is None:
            position = [-2, -2, 0.001]
        if orientation is None:
            orientation = [0, 0, 0, 1]
        if self.multiverse.robot is None or replace:
            if self.multiverse.robot is not None:
                self.multiverse.robot.remove()
            robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}.urdf",
                           pose=Pose(position, [0, 0, 0, 1]))
        else:
            robot = self.multiverse.robot
            robot.set_position(position)
        return robot

    @staticmethod
    def spawn_cup(position: List) -> Object:
        cup = Object("cup", ObjectType.GENERIC_OBJECT, "Cup.obj",
                     pose=Pose(position, [0, 0, 0, 1]))
        return cup

    def assert_poses_are_equal(self, pose1: Pose, pose2: Pose,
                               position_delta: Optional[float] = None, orientation_delta: Optional[float] = None):
        if position_delta is None:
            position_delta = self.multiverse.conf.position_tolerance
        if orientation_delta is None:
            orientation_delta = self.multiverse.conf.orientation_tolerance
        self.assert_position_is_equal(pose1.position_as_list(), pose2.position_as_list(), delta=position_delta)
        self.assert_orientation_is_equal(pose1.orientation_as_list(), pose2.orientation_as_list(),
                                         delta=orientation_delta)

    def assert_position_is_equal(self, position1: List[float], position2: List[float], delta: Optional[float] = None):
        if delta is None:
            delta = self.multiverse.conf.position_tolerance
        self.assert_list_is_equal(position1, position2, delta=delta)

    def assert_orientation_is_equal(self, orientation1: List[float], orientation2: List[float],
                                    delta: Optional[float] = None):
        if delta is None:
            delta = self.multiverse.conf.orientation_tolerance
        self.assertAlmostEqual(calculate_angle_between_quaternions(orientation1, orientation2), 0, delta=delta)

    def assert_list_is_equal(self, list1: List, list2: List, delta: float):
        for i in range(len(list1)):
            self.assertAlmostEqual(list1[i], list2[i], delta=delta)
