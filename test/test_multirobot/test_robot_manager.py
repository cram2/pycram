from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.multirobot import RobotManager
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.robot_description import RobotDescription
from pycram.testing import EmptyBulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Robot, Milk, Cereal


class MultiRobotTestCase(EmptyBulletWorldTestCase):
    world: BulletWorld
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.world = BulletWorld(mode=WorldMode.DIRECT)

        cls.robot_pr2 = Object("pr2", Robot,
                               'pr2' + cls.extension,
                               pose=PoseStamped.from_list([0, 1, 0]))

        cls.robot_tiago = Object("tiago_dual", Robot,
                                 "tiago_dual" + cls.extension,
                                 pose=PoseStamped.from_list([0, 3, 0]))

        cls.milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([0.5, 3, 1.02]))
        cls.table = Object("cereal", Cereal, "breakfast_cereal.stl", pose=PoseStamped.from_list([1.5, 4, 1.02]))

    def check_robot(self, name, base_link, torso_link, torso_joint, number_of_links, number_of_joints):
        self.assertEqual(RobotDescription.current_robot_description.name, name)
        self.assertEqual(RobotDescription.current_robot_description.base_link, base_link)
        self.assertEqual(RobotDescription.current_robot_description.torso_link, torso_link)
        self.assertEqual(RobotDescription.current_robot_description.torso_joint, torso_joint)
        self.assertEqual(len(RobotDescription.current_robot_description.links), number_of_links)
        self.assertEqual(len(RobotDescription.current_robot_description.joints), number_of_joints)

    def check_current_robot(self, name):
        if name == "pr2":
            self.check_robot(name="pr2", base_link="base_link", torso_link="torso_lift_link",
                             torso_joint="torso_lift_joint", number_of_links=88, number_of_joints=87)
        elif name == "tiago_dual":
            self.check_robot(name="tiago_dual", base_link="base_link", torso_link="torso_lift_link",
                             torso_joint="torso_lift_joint", number_of_links=69, number_of_joints=68)


class TestMultiRobot(MultiRobotTestCase):

    def test_load_multiple_robots(self):
        RobotManager.set_active_robot(self.robot_pr2.name)

        self.check_current_robot(self.robot_pr2.name)

        RobotManager.set_active_robot(self.robot_tiago.name)

        self.check_current_robot(self.robot_tiago.name)
