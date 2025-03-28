import time
import unittest
import xml.etree.ElementTree as ET

from pycram.testing import BulletWorldTestCase, BulletWorldGUITestCase
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.world import UseProspectionWorld
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.robot_description import RobotDescription
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Milk, Robot
from pycram.ros import get_ros_package_path

fix_missing_inertial = ObjectDescription.fix_missing_inertial


class BulletWorldTest(BulletWorldTestCase):

    def test_object_movement(self):
        self.milk.set_position(PoseSteamped.from_list([0, 1, 1]))
        self.assertEqual(self.milk.get_position_as_list(), [0, 1, 1])

    def test_robot_orientation(self):
        self.robot.set_pose(PoseSteamped.from_list([0, 1, 1]))
        head_position = self.robot.get_link_position('head_pan_link').z
        self.robot.set_orientation(PoseStamped(orientation=[0, 0, 1, 1]))
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
        BulletWorldTest.milk = Object("milk", Milk, "milk.stl", pose=PoseSteamped.from_list([1.3, 1, 0.9]))

    def test_remove_robot(self):
        robot_id = self.robot.id
        self.assertTrue(robot_id in [obj.id for obj in self.world.objects])
        self.world.remove_object(self.robot)
        self.assertTrue(robot_id not in [obj.id for obj in self.world.objects])
        BulletWorldTest.robot = Object(RobotDescription.current_robot_description.name, Robot,
                                       RobotDescription.current_robot_description.name + self.extension)

    def test_get_joint_position(self):
        self.assertAlmostEqual(self.robot.get_joint_position("head_pan_joint"), 0.0, delta=0.01)

    def test_get_object_contact_points(self):
        self.assertEqual(len(self.robot.contact_points), 0)  # 8 because of robot wheels with floor
        self.milk.set_position(self.robot.get_position())
        self.assertTrue(len(self.robot.contact_points) > 0)

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
        self.milk.set_position(PoseSteamped.from_list([0, 0, 2]))
        self.world.simulate(1)
        self.assertTrue(self.milk.get_position().z < 2)

    @unittest.skip
    def test_set_real_time_simulation(self):
        self.milk.set_position(PoseSteamped.from_list([100, 0, 2]))
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
        self.robot.set_pose(PoseSteamped.from_list([1, 0, 0], [0, 0, 0, 1]))
        self.assertFalse(self.world.world_sync.check_for_equal())
        with UseProspectionWorld():
            self.assertTrue(self.world.world_sync.check_for_equal())

    def test_add_resource_path(self):
        self.world.add_resource_path("test")
        self.assertTrue("test" in self.world.get_data_directories())

    def test_no_prospection_object_found_for_given_object(self):
        milk_2 = Object("milk_2", Milk, "milk.stl", pose=PoseSteamped.from_list([1.3, 1, 0.9]))
        try:
            prospection_milk_2 = self.world.get_prospection_object_for_object(milk_2)
            self.world.remove_object(milk_2)
            self.world.get_prospection_object_for_object(milk_2)
            self.assertFalse(True)
        except KeyError as e:
            self.assertTrue(True)

    def test_real_object_position_does_not_change_with_prospection_object(self):
        milk_2_pos = [1.3, 1, 0.9]
        milk_2 = Object("milk_3", Milk, "milk.stl", pose=PoseStamped(milk_2_pos))
        time.sleep(0.05)
        milk_2_pos = milk_2.get_position()

        # Assert that when prospection object is moved, the real object is not moved
        with UseProspectionWorld():
            prospection_milk = self.world.get_prospection_object_for_object(milk_2)
            prospection_milk_pos = prospection_milk.get_position()
            self.assertTrue(prospection_milk_pos == milk_2_pos)
            prospection_milk_pos.x += 1
            prospection_milk.set_position(prospection_milk_pos)
            self.assertTrue(prospection_milk.get_position() != milk_2.get_position())
        self.world.remove_object(milk_2)

    def test_prospection_object_position_does_not_change_with_real_object(self):
        milk_2_pos = [1.3, 1, 0.9]
        milk_2 = Object("milk_4", Milk, "milk.stl", pose=PoseStamped(milk_2_pos))
        time.sleep(0.05)
        milk_2_pos = milk_2.get_position()

        # Assert that when real object is moved, the prospection object is not moved
        with UseProspectionWorld():
            prospection_milk = self.world.get_prospection_object_for_object(milk_2)
            prospection_milk_pos = prospection_milk.get_position()
            self.assertTrue(prospection_milk_pos == milk_2_pos)
            milk_2_pos.x += 1
            milk_2.set_position(milk_2_pos)
            self.assertTrue(prospection_milk.get_position() != milk_2.get_position())
        self.world.remove_object(milk_2)

    def test_add_vis_axis(self):
        self.world.add_vis_axis(self.robot.get_link_pose(RobotDescription.current_robot_description.get_camera_link()))
        self.assertTrue(len(self.world.vis_axis) == 1)
        self.world.remove_vis_axis()
        self.assertTrue(len(self.world.vis_axis) == 0)

    def test_add_text(self):
        link: ObjectDescription.Link = self.robot.get_link(RobotDescription.current_robot_description.get_camera_link())
        text_id = self.world.add_text("test", link.position.to_list(), link.orientation.to_list(), 1,
                                      Color(1, 0, 0, 1), 3, link.object_id, link.id)
        if self.world.mode == WorldMode.GUI:
            time.sleep(4)

    def test_remove_text(self):
        link: ObjectDescription.Link = self.robot.get_link(RobotDescription.current_robot_description.get_camera_link())
        text_id_1 = self.world.add_text("test 1", link.pose.position.to_list(), link.pose.orientation.to_list(), 1,
                                        Color(1, 0, 0, 1), 0, link.object_id, link.id)
        text_id = self.world.add_text("test 2", link.pose.position.to_list(), link.pose.orientation.to_list(), 1,
                                      Color(0, 1, 0, 1), 0, link.object_id, link.id)

        if self.world.mode == WorldMode.GUI:
            time.sleep(2)
        self.world.remove_text(text_id_1)
        if self.world.mode == WorldMode.GUI:
            time.sleep(3)

    def test_remove_all_text(self):
        link: ObjectDescription.Link = self.robot.get_link(RobotDescription.current_robot_description.get_camera_link())
        text_id_1 = self.world.add_text("test 1", link.pose.position.to_list(), link.pose.orientation.to_list(), 1,
                                        Color(1, 0, 0, 1), 0, link.object_id, link.id)
        text_id = self.world.add_text("test 2", link.pose.position.to_list(), link.pose.orientation.to_list(), 1,
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
        self.world.add_vis_axis(self.robot.get_link_pose(RobotDescription.current_robot_description.get_camera_link()))
        self.assertTrue(len(self.world.vis_axis) == 1)
        self.world.remove_vis_axis()
        self.assertTrue(len(self.world.vis_axis) == 0)


class XMLTester(unittest.TestCase):

    def setUp(self) -> None:
        filename = get_ros_package_path('pycram') + '/resources/robots/' + 'pr2.urdf'
        with open(filename, "r") as f:
            self.urdf_string = f.read()

    def test_inertial(self):
        result = fix_missing_inertial(self.urdf_string)
        resulting_tree = ET.ElementTree(ET.fromstring(result))
        for element in resulting_tree.iter("link"):
            self.assertTrue(len([*element.iter("inertial")]) > 0)
