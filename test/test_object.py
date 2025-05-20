import numpy as np
import trimesh.parent
from pycram.tf_transformations import quaternion_from_euler

from pycram.testing import BulletWorldTestCase

from pycram.datastructures.enums import JointType, ObjectType
from pycram.datastructures.pose import PoseStamped, Point, Quaternion
from pycram.datastructures.dataclasses import Color, BoundingBox as BB
from pycram.failures import UnsupportedFileExtension
from pycram.world_concepts.world_object import Object
from pycram.object_descriptors.generic import ObjectDescription as GenericObjectDescription

import pathlib

from pycrap.ontologies import Milk, Food


class TestObject(BulletWorldTestCase):

    def test_wrong_object_description_path(self):
        with self.assertRaises(UnsupportedFileExtension):
            milk = Object("milk_not_found", Milk, "wrong_path.sk")

    def test_malformed_object_description(self):
        file_path = pathlib.Path(__file__).parent.resolve()
        malformed_file = str(file_path) + "/../resources/cached/malformed_description.urdf"
        with open(malformed_file, "w") as file:
            file.write("malformed")
        with self.assertRaises(Exception):
            Object("milk2", Milk, malformed_file)

    def test_move_base_to_origin_pose(self):
        self.milk.set_position(Point(x=1, y=2, z=3), base=False)
        self.milk.move_base_to_origin_pose()
        self.assertEqual(self.milk.get_base_position_as_list(), [1, 2, 3])

    def test_set_position_as_point(self):
        self.milk.set_position(Point(x=1, y=2, z=3))
        self.assertEqual(self.milk.get_position_as_list(), [1, 2, 3])

    def test_uni_direction_attachment(self):
        self.milk.attach(self.cereal, bidirectional=False)

        milk_position = self.milk.get_position_as_list()
        cereal_position = self.cereal.get_position_as_list()

        def move_milk_and_assert_cereal_moved():
            milk_position[0] += 1
            cereal_position[0] += 1
            self.milk.set_position(milk_position)

            new_cereal_position = self.cereal.get_position_as_list()
            self.assertEqual(new_cereal_position, cereal_position)

        def move_cereal_and_assert_milk_not_moved():
            cereal_position[0] += 1
            self.cereal.set_position(cereal_position)

            new_milk_position = self.milk.get_position_as_list()
            self.assertEqual(new_milk_position, milk_position)

        # move twice to test updated attachment at the new cereal position
        for _ in range(2):
            move_milk_and_assert_cereal_moved()
            move_cereal_and_assert_milk_not_moved()

    def test_setting_wrong_position_type(self):
        with self.assertRaises(TypeError):
            self.milk.set_position(1)

        with self.assertRaises(AssertionError):
            self.assertEqual(1, self.milk.pose.position)

    def test_set_orientation_as_list(self):
        self.milk.set_orientation([1, 0, 0, 0])
        self.assertEqual(self.milk.get_orientation_as_list(), [1, 0, 0, 0])

    def test_set_orientation_as_quaternion(self):
        self.milk.set_orientation(Quaternion(**dict(zip(["x", "y", "z", "w"], [1, 0, 0, 0]))))
        self.assertEqual(self.milk.get_orientation_as_list(), [1, 0, 0, 0])

    def test_set_orientation_as_ndarray(self):
        self.milk.set_orientation(np.array([1, 0, 0, 0]))
        self.assertEqual(self.milk.get_orientation_as_list(), [1, 0, 0, 0])

    def test_set_wrong_orientation_type(self):
        with self.assertRaises(TypeError):
            self.milk.set_orientation(1)

    def test_set_position_as_pose(self):
        self.milk.set_position(PoseStamped.from_list([1, 2, 3]))
        self.assertEqual(self.milk.get_position_as_list(), [1, 2, 3])

    def test_set_position_as_list(self):
        self.milk.set_position([1, 2, 3])
        self.assertEqual(self.milk.get_position_as_list(), [1, 2, 3])

    def test_get_joint_axis(self):
        self.assertEqual(self.robot.get_joint_axis("head_pan_joint"), Point(x=0.0, y=0.0, z=1.0))

    def test_get_joint_type(self):
        self.assertEqual(self.robot.get_joint_type("head_pan_joint"), JointType.REVOLUTE)

    def test_get_joint_lower_limit(self):
        self.assertEqual(self.robot.get_joint_lower_limit("head_pan_joint"), -3.007)

    def test_get_joint_upper_limit(self):
        self.assertEqual(self.robot.get_joint_upper_limit("head_pan_joint"), 3.007)

    def test_get_joint_damping(self):
        self.assertEqual(self.robot.get_joint_damping("head_pan_joint"), 0.5)

    def test_save_state(self):
        self.robot.attach(self.milk)
        self.robot.save_state(1)
        self.assertEqual(self.robot.saved_states[1].attachments, self.robot.attachments)
        self.assertTrue(self.milk in self.robot.saved_states[1].attachments)
        for link in self.robot.links.values():
            self.assertEqual(link.current_state, link.saved_states[1])

    def test_restore_state(self):
        self.robot.attach(self.milk)
        self.robot.save_state(1)
        self.milk.save_state(1)
        self.assertTrue(self.milk in self.robot.attachments)
        self.robot.detach(self.milk)
        self.assertTrue(self.milk not in self.robot.attachments)
        self.robot.restore_state(1)
        self.milk.restore_state(1)
        self.assertTrue(self.milk in self.robot.attachments)
        for link in self.robot.links.values():
            curr_state = link.current_state
            saved_state = link.saved_states[1]
            self.assertEqual(curr_state, saved_state)
            self.assertEqual(curr_state.constraint_ids, saved_state.constraint_ids)

    def test_get_link_by_id(self):
        self.assertEqual(self.robot.get_link_by_id(-1), self.robot.root_link)

    def test_find_joint_above_link(self):
        self.assertEqual(self.robot.find_joint_above_link("head_pan_link"), "head_pan_joint")

    def test_find_joint_above_link_exhausted(self):
        self.assertEqual(self.robot.find_joint_above_link("base_link"), None)

    def test_contact_points_simulated(self):
        self.milk.set_position([0, 0, 100])
        contact_points = self.milk.contact_points_simulated()
        self.assertFalse(contact_points)
        self.milk.set_position(self.cereal.get_position_as_list(), base=True)
        contact_points = self.milk.contact_points_simulated()
        self.assertTrue(contact_points)

    def test_set_color(self):
        self.milk.set_color(Color(1, 0, 0, 1))
        self.assertEqual(self.milk.get_color(), Color(1, 0, 0, 1))
        self.robot.set_color(Color(0, 1, 0, 1))
        for color in self.robot.get_color().values():
            self.assertEqual(color, Color(0, 1, 0, 1))

    def test_object_equal(self):
        milk2 = Object("milk2", Milk, "milk.stl")
        self.assertNotEqual(self.milk, milk2)
        self.assertEqual(self.milk, self.milk)
        self.assertNotEqual(self.milk, self.cereal)
        self.assertNotEqual(self.milk, self.world)

    def test_merge(self):
        cereal = Object("cereal2", Food, "breakfast_cereal.stl")
        milk = Object("milk2", Milk, "milk.stl")
        cereal_milk = cereal.merge(milk, new_description_file="cereal_milk")
        self.assertEqual(len(cereal_milk.links), len(cereal.links) + len(milk.links))
        self.assertEqual(len(cereal_milk.joints), len(cereal.joints) + len(milk.joints) + 1)
        cereal_milk.remove()
        # self.world.cache_manager.clear_cache()

    def test_merge_bounding_box(self):
        cereal_2 = Object("cereal2", Food, "breakfast_cereal.stl",
                          pose=self.cereal.pose)
        cereal_2.set_orientation(quaternion_from_euler(0, 0, np.pi / 2))
        cereal_bbox = self.cereal.get_axis_aligned_bounding_box(False)
        cereal_2_bbox = cereal_2.get_axis_aligned_bounding_box(False)
        plot = False
        if plot:
            BB.plot_3d_points([np.array(cereal_bbox.get_points_list()), np.array(cereal_2_bbox.get_points_list())])
        for use_random_events in [True, False]:
            merged_bbox_mesh = BB.merge_multiple_bounding_boxes_into_mesh([cereal_bbox, cereal_2_bbox],
                                                                          use_random_events=use_random_events,
                                                                          plot=plot)
            self.assertTrue(isinstance(merged_bbox_mesh, trimesh.Trimesh))
            self.assertTrue(merged_bbox_mesh.vertices.shape[0] > 0)
            self.assertTrue(merged_bbox_mesh.faces.shape[0] > 0)
            self.assertTrue(merged_bbox_mesh.is_volume if not use_random_events else True)
            self.assertTrue(merged_bbox_mesh.volume > 0 if not use_random_events else True)
        cereal_2.remove()


class GenericObjectTestCase(BulletWorldTestCase):

    def test_init_generic_object(self):
        gen_obj_desc = GenericObjectDescription("robokudo_object", [0,0,0], [0.1, 0.1, 0.1])
        obj = Object("robokudo_object", Milk, None, gen_obj_desc)
        pose = obj.get_pose()
        self.assertTrue(isinstance(pose, PoseStamped))


class OntologyIntegrationTestCase(BulletWorldTestCase):

    def test_querying(self):
        """
         Test case if spawning a second object of the same Concept in this case milk,
         Since the Object Link Description contains the same name for every object coming from the same STL-File,
         it is required to add a unique id for every new parsed link description.
         We spawn a second milk object, so there should be two instances of type milk. """
        milk2 = Object("milk2", Milk, "milk.stl")
        self.world.ontology.reason()
        r = list(filter(lambda x:  x in Milk.instances(), self.world.ontology.individuals()))
        self.assertEqual(len(r), 2)
