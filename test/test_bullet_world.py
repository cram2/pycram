import time
import unittest
import xml.etree.ElementTree as ET

import rospkg

from bullet_world_testcase import BulletWorldTestCase, BulletWorldGUITestCase
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose
from pycram.robot_description import RobotDescription
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.datastructures.dataclasses import Color
from pycram.world_concepts.world_object import Object
from pycram.datastructures.world import UseProspectionWorld

fix_missing_inertial = ObjectDescription.fix_missing_inertial


class BulletWorldTest(BulletWorldTestCase):

    def test_object_movement(self):
        self.milk.set_position(Pose([0, 1, 1]))
        self.assertEqual(self.milk.get_position_as_list(), [0, 1, 1])

    def test_robot_orientation(self):
        self.robot.set_pose(Pose([0, 1, 1]))
        head_position = self.robot.get_link_position('head_pan_link').z
        self.robot.set_orientation(Pose(orientation=[0, 0, 1, 1]))
        self.assertEqual(self.robot.get_link_position('head_pan_link').z, head_position)

    def test_save_and_restore_state(self):
        self.robot.attach(self.milk)
        self.milk.attach(self.cereal)
        all_object_attachments = {obj: obj.attachments.copy() for obj in self.world.objects}
        state_id = self.world.save_state()
        self.milk.detach(self.cereal)
        robot_link = self.robot.root_link
        milk_link = self.milk.root_link
        cid = robot_link.constraint_ids[milk_link]
        self.assertTrue(cid == self.robot.attachments[self.milk].id)
        self.world.remove_constraint(cid)
        self.world.restore_state(state_id)
        cid = robot_link.constraint_ids[milk_link]
        self.assertTrue(milk_link in robot_link.constraint_ids)
        self.assertTrue(cid == self.robot.attachments[self.milk].id)
        for obj in self.world.objects:
            self.assertTrue(len(obj.attachments) == len(all_object_attachments[obj]))
            for att in obj.attachments:
                self.assertTrue(att in all_object_attachments[obj])

    def test_remove_object(self):
        milk_id = self.milk.id
        self.assertTrue(milk_id in [obj.id for obj in self.world.objects])
        self.world.remove_object(self.milk)
        self.assertTrue(milk_id not in [obj.id for obj in self.world.objects])
        BulletWorldTest.milk = Object("milk", ObjectType.MILK, "milk.stl",
                                      ObjectDescription, pose=Pose([1.3, 1, 0.9]))

    def test_remove_robot(self):
        robot_id = self.robot.id
        self.assertTrue(robot_id in [obj.id for obj in self.world.objects])
        self.world.remove_object(self.robot)
        self.assertTrue(robot_id not in [obj.id for obj in self.world.objects])
        BulletWorldTest.robot = Object(RobotDescription.current_robot_description.name, ObjectType.ROBOT,
                                       RobotDescription.current_robot_description.name + self.extension)

    def test_get_joint_position(self):
        self.assertEqual(self.robot.get_joint_position("head_pan_joint"), 0.0)

    def test_get_object_contact_points(self):
        self.assertEqual(len(self.robot.contact_points()), 0)
        self.milk.set_position(self.robot.get_position())
        self.assertTrue(len(self.robot.contact_points()) > 0)

    def test_enable_joint_force_torque_sensor(self):
        self.world.enable_joint_force_torque_sensor(self.robot, self.robot.get_joint_id("head_pan_joint"))
        force_torque = self.world.get_joint_reaction_force_torque(self.robot, self.robot.get_joint_id("head_pan_joint"))
        # TODO: useless because even if the sensor is disabled, the force_torque is still 0
        for ft in force_torque:
            self.assertTrue(ft == 0.0)

    def test_disable_joint_force_torque_sensor(self):
        self.world.enable_joint_force_torque_sensor(self.robot, self.robot.get_joint_id("head_pan_joint"))
        self.world.disable_joint_force_torque_sensor(self.robot, self.robot.get_joint_id("head_pan_joint"))
        force_torque = self.world.get_joint_reaction_force_torque(self.robot, self.robot.get_joint_id("head_pan_joint"))
        for ft in force_torque:
            self.assertTrue(ft == 0.0)

    def test_get_applied_joint_motor_torque(self):
        self.world.get_applied_joint_motor_torque(self.robot, self.robot.get_joint_id("head_pan_joint"))

    def test_step_simulation(self):
        # TODO: kitchen explodes when stepping simulation, fix this
        self.kitchen.set_position([100, 100, 0])
        self.milk.set_position(Pose([0, 0, 2]))
        self.world.simulate(1)
        self.assertTrue(self.milk.get_position().z < 2)

    @unittest.skip
    def test_set_real_time_simulation(self):
        self.milk.set_position(Pose([100, 0, 2]))
        curr_time = time.time()
        self.world.simulate(0.5, real_time=True)
        time_elapsed = time.time() - curr_time
        self.assertAlmostEqual(time_elapsed, 0.5, delta=0.2)

    def test_collision_callback(self):
        self.kitchen.set_position([100, 100, 0])
        self.collision_called = False
        self.no_collision_called = False

        def collision_callback():
            self.collision_called = True

        def no_collision_callback():
            self.no_collision_called = True

        self.world.register_two_objects_collision_callbacks(self.milk, self.cereal,
                                                            collision_callback, no_collision_callback)

        self.world.simulate(1)
        self.assertTrue(self.no_collision_called)
        self.assertFalse(self.collision_called)

        self.collision_called = False
        self.no_collision_called = False

        new_milk_position = self.cereal.get_position()
        new_milk_position.z += 0.5
        self.milk.set_position(new_milk_position)

        self.world.simulate(4)
        self.assertTrue(self.collision_called)

    def test_equal_world_states(self):
        time.sleep(2.5)
        self.robot.set_pose(Pose([1, 0, 0], [0, 0, 0, 1]))
        self.assertFalse(self.world.world_sync.check_for_equal())
        self.world.prospection_world.object_states = self.world.current_state.object_states
        time.sleep(0.05)
        self.assertTrue(self.world.world_sync.check_for_equal())

    def test_add_resource_path(self):
        self.world.add_resource_path("test")
        self.assertTrue("test" in self.world.data_directory)

    def test_no_prospection_object_found_for_given_object(self):
        milk_2 = Object("milk_2", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        time.sleep(0.05)
        try:
            prospection_milk_2 = self.world.get_prospection_object_for_object(milk_2)
            self.world.remove_object(milk_2)
            time.sleep(0.1)
            self.world.get_prospection_object_for_object(milk_2)
            self.assertFalse(True)
        except ValueError as e:
            self.assertTrue(True)

    def test_no_object_found_for_given_prospection_object(self):
        milk_2 = Object("milk_2", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        time.sleep(0.05)
        prospection_milk = self.world.get_prospection_object_for_object(milk_2)
        self.assertTrue(self.world.get_object_for_prospection_object(prospection_milk) == milk_2)
        try:
            self.world.remove_object(milk_2)
            self.world.get_object_for_prospection_object(prospection_milk)
            time.sleep(0.1)
            self.assertFalse(True)
        except ValueError as e:
            self.assertTrue(True)
        time.sleep(0.05)

    def test_real_object_position_does_not_change_with_prospection_object(self):
        milk_2_pos = [1.3, 1, 0.9]
        milk_2 = Object("milk_3", ObjectType.MILK, "milk.stl", pose=Pose(milk_2_pos))
        time.sleep(0.05)
        milk_2_pos = milk_2.get_position()
        prospection_milk = self.world.get_prospection_object_for_object(milk_2)
        prospection_milk_pos = prospection_milk.get_position()
        self.assertTrue(prospection_milk_pos == milk_2_pos)

        # Assert that when prospection object is moved, the real object is not moved
        with UseProspectionWorld():
            prospection_milk_pos.x += 1
            prospection_milk.set_position(prospection_milk_pos)
            self.assertTrue(prospection_milk.get_position() != milk_2.get_position())
        self.world.remove_object(milk_2)

    def test_prospection_object_position_does_not_change_with_real_object(self):
        milk_2_pos = [1.3, 1, 0.9]
        milk_2 = Object("milk_4", ObjectType.MILK, "milk.stl", pose=Pose(milk_2_pos))
        time.sleep(0.05)
        milk_2_pos = milk_2.get_position()
        prospection_milk = self.world.get_prospection_object_for_object(milk_2)
        prospection_milk_pos = prospection_milk.get_position()
        self.assertTrue(prospection_milk_pos == milk_2_pos)

        # Assert that when real object is moved, the prospection object is not moved
        with UseProspectionWorld():
            milk_2_pos.x += 1
            milk_2.set_position(milk_2_pos)
            self.assertTrue(prospection_milk.get_position() != milk_2.get_position())
        self.world.remove_object(milk_2)

    def test_add_vis_axis(self):
        self.world.add_vis_axis(self.robot.get_link_pose(RobotDescription.current_robot_description.get_camera_frame()))
        self.assertTrue(len(self.world.vis_axis) == 1)
        self.world.remove_vis_axis()
        self.assertTrue(len(self.world.vis_axis) == 0)

    def test_add_text(self):
        link: ObjectDescription.Link = self.robot.get_link(RobotDescription.current_robot_description.get_camera_frame())
        text_id = self.world.add_text("test", link.position_as_list, link.orientation_as_list, 1,
                                      Color(1, 0, 0, 1), 3, link.object_id, link.id)
        if self.world.mode == WorldMode.GUI:
            time.sleep(4)

    def test_remove_text(self):
        link: ObjectDescription.Link = self.robot.get_link(RobotDescription.current_robot_description.get_camera_frame())
        text_id_1 = self.world.add_text("test 1", link.pose.position_as_list(), link.pose.orientation_as_list(), 1,
                                        Color(1, 0, 0, 1), 0, link.object_id, link.id)
        text_id = self.world.add_text("test 2", link.pose.position_as_list(), link.pose.orientation_as_list(), 1,
                                      Color(0, 1, 0, 1), 0, link.object_id, link.id)

        if self.world.mode == WorldMode.GUI:
            time.sleep(2)
        self.world.remove_text(text_id_1)
        if self.world.mode == WorldMode.GUI:
            time.sleep(3)

    def test_remove_all_text(self):
        link: ObjectDescription.Link = self.robot.get_link(RobotDescription.current_robot_description.get_camera_frame())
        text_id_1 = self.world.add_text("test 1", link.pose.position_as_list(), link.pose.orientation_as_list(), 1,
                                        Color(1, 0, 0, 1), 0, link.object_id, link.id)
        text_id = self.world.add_text("test 2", link.pose.position_as_list(), link.pose.orientation_as_list(), 1,
                                      Color(0, 1, 0, 1), 0, link.object_id, link.id)
        if self.world.mode == WorldMode.GUI:
            time.sleep(2)
        self.world.remove_text()
        if self.world.mode == WorldMode.GUI:
            time.sleep(3)


@unittest.skip("Not working in CI")
class BulletWorldTestGUI(BulletWorldGUITestCase):
    def test_add_vis_axis(self):
        time.sleep(10)
        self.world.add_vis_axis(self.robot.get_link_pose(RobotDescription.current_robot_description.get_camera_frame()))
        self.assertTrue(len(self.world.vis_axis) == 1)
        self.world.remove_vis_axis()
        self.assertTrue(len(self.world.vis_axis) == 0)


class XMLTester(unittest.TestCase):

    def setUp(self) -> None:
        rospack = rospkg.RosPack()
        filename = rospack.get_path('pycram') + '/resources/robots/' + 'pr2.urdf'
        with open(filename, "r") as f:
            self.urdf_string = f.read()

    def test_inertial(self):
        result = fix_missing_inertial(self.urdf_string)
        resulting_tree = ET.ElementTree(ET.fromstring(result))
        for element in resulting_tree.iter("link"):
            self.assertTrue(len([*element.iter("inertial")]) > 0)
