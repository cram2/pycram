import time

from pycram.testing import BulletWorldTestCase
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.world import UseProspectionWorld
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Milk, Cereal


class TestAttachment(BulletWorldTestCase):

    def test_attach(self):
        self.milk.attach(self.robot)
        self.assertTrue(self.milk.attachments[self.robot])
        self.assertTrue(self.robot.attachments[self.milk])

    def test_detach(self):
        self.milk.attach(self.robot)
        self.milk.detach(self.robot)
        self.assertTrue(self.robot not in self.milk.attachments)
        self.assertTrue(self.milk not in self.robot.attachments)

    def test_detach_sync_in_prospection_world(self):
        self.milk.attach(self.robot)
        with UseProspectionWorld():
            pass
        self.milk.detach(self.robot)
        with UseProspectionWorld():
            pass
        self.assertTrue(self.milk not in self.robot.attachments)
        self.assertTrue(self.robot not in self.milk.attachments)
        prospection_milk = self.world.get_prospection_object_for_object(self.milk)
        prospection_robot = self.world.get_prospection_object_for_object(self.robot)
        self.assertTrue(prospection_milk not in prospection_robot.attachments)
        self.assertTrue(prospection_robot not in prospection_milk.attachments)

    def test_attachment_behavior(self):
        self.robot.attach(self.milk)

        milk_pos = self.milk.get_position()
        rob_pos = self.robot.get_position()

        rob_pos.x += 1
        self.robot.set_position(rob_pos)

        new_milk_pos = self.milk.get_position()
        self.assertEqual(new_milk_pos.x, milk_pos.x + 1)

    def test_detachment_behavior(self):
        self.robot.attach(self.milk)

        milk_pos = self.milk.get_position()
        rob_pos = self.robot.get_position()

        self.robot.detach(self.milk)
        rob_pos.x += 1
        self.robot.set_position(rob_pos)

        new_milk_pos = self.milk.get_position()
        self.assertEqual(new_milk_pos.x, milk_pos.x)

    def test_prospection_object_attachments_not_changed_with_real_object(self):
        milk_2 = Object("milk_2", Milk, "milk.stl", pose=PoseStamped.from_list([1.3, 1, 0.9]))
        cereal_2 = Object("cereal_2", Cereal, "breakfast_cereal.stl",
                          pose=PoseStamped.from_list([1.3, 0.7, 0.95]))
        time.sleep(0.05)
        milk_2.attach(cereal_2)
        time.sleep(0.05)
        with UseProspectionWorld():
            prospection_milk = self.world.get_prospection_object_for_object(milk_2)
            # self.assertTrue(cereal_2 not in prospection_milk.attachments)
            prospection_cereal = self.world.get_prospection_object_for_object(cereal_2)
            # self.assertTrue(prospection_cereal in prospection_milk.attachments)
            self.assertTrue(prospection_cereal in prospection_milk.attachments.keys())

            # Assert that when prospection object is moved, the real object is not moved
            prospection_milk_pos = prospection_milk.get_position()
            cereal_pos = cereal_2.get_position()
            estimated_prospection_cereal_pos = prospection_cereal.get_position()
            estimated_prospection_cereal_pos.x += 1

            # Move prospection milk object
            prospection_milk_pos.x += 1
            prospection_milk.set_position(prospection_milk_pos)

            # Prospection cereal should move since it is attached to prospection milk
            new_prospection_cereal_pose = prospection_cereal.get_position()
            self.assertAlmostEqual(new_prospection_cereal_pose.x, estimated_prospection_cereal_pos.x, delta=0.01)

            # Also Real cereal object should not move since it is not affected by prospection milk
            new_cereal_pos = cereal_2.get_position()
            assumed_cereal_pos = cereal_pos
            self.assertTrue(new_cereal_pos == assumed_cereal_pos)

        self.world.remove_object(milk_2)
        self.world.remove_object(cereal_2)

    def test_attaching_to_robot_and_moving(self):
        self.robot.attach(self.milk)
        milk_pos = self.milk.get_position()
        rob_pos = self.robot.get_position()

        rob_pos.x += 1
        self.robot.set_position(rob_pos)

        new_milk_pos = self.milk.get_position()
        self.assertEqual(new_milk_pos.x, milk_pos.x + 1)
