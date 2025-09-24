from semantic_world.adapters.viz_marker import VizMarkerPublisher

from pycram.designator import ObjectDesignatorDescription
from pycram.designators.location_designator import *
from pycram.language import SequentialPlan
from pycram.robot_description import RobotDescription
from pycram.datastructures.pose import PoseStamped
from pycram.robot_plans import NavigateActionDescription
from pycram.ros import node
from pycram.testing import BulletWorldTestCase

class TestActionDesignatorGrounding(BulletWorldTestCase):

    def test_reachability_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                         'l_shoulder_lift_joint': -0.264,
                         'l_upper_arm_roll_joint': 1.38,
                         'l_elbow_flex_joint': -2.12,
                         'l_forearm_roll_joint': 16.996,
                         'l_wrist_flex_joint': -0.073,
                         'l_wrist_roll_joint': 0.0,
                        'r_shoulder_pan_joint': -1.712,
                          'r_shoulder_lift_joint': -0.256,
                          'r_upper_arm_roll_joint': -1.463,
                          'r_elbow_flex_joint': -2.12,
                          'r_forearm_roll_joint': 1.766,
                          'r_wrist_flex_joint': -0.07,
                          'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position = 0.3
        self.world.notify_state_change()

        location_desig = CostmapLocation(self.world.get_body_by_name("milk.stl"), reachable_for=self.robot_view)
        plan = SequentialPlan((self.world, None), self.robot_view, NavigateActionDescription(location_desig))
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)
        #self.assertTrue(Arms.LEFT == location.reachable_arm or Arms.RIGHT == location.reachable_arm)

    def test_reachability_pose_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                         'l_shoulder_lift_joint': -0.264,
                         'l_upper_arm_roll_joint': 1.38,
                         'l_elbow_flex_joint': -2.12,
                         'l_forearm_roll_joint': 16.996,
                         'l_wrist_flex_joint': -0.073,
                         'l_wrist_roll_joint': 0.0,
                        'r_shoulder_pan_joint': -1.712,
                          'r_shoulder_lift_joint': -0.256,
                          'r_upper_arm_roll_joint': -1.463,
                          'r_elbow_flex_joint': -2.12,
                          'r_forearm_roll_joint': 1.766,
                          'r_wrist_flex_joint': -0.07,
                          'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        self.world.notify_state_change()
        location_desig = CostmapLocation(PoseStamped.from_list(self.world.root, [0.4, 0.6, 0.9], [0, 0, 0, 1]), reachable_for=self.robot_view)
        plan = SequentialPlan(self.context, self.robot_view, NavigateActionDescription(location_desig))
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)
        #self.assertTrue(Arms.LEFT == location.reachable_arm or Arms.RIGHT == location.reachable_arm)

    def test_visibility_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                         'l_shoulder_lift_joint': -0.264,
                         'l_upper_arm_roll_joint': 1.38,
                         'l_elbow_flex_joint': -2.12,
                         'l_forearm_roll_joint': 16.996,
                         'l_wrist_flex_joint': -0.073,
                         'l_wrist_roll_joint': 0.0,
                        'r_shoulder_pan_joint': -1.712,
                          'r_shoulder_lift_joint': -0.256,
                          'r_upper_arm_roll_joint': -1.463,
                          'r_elbow_flex_joint': -2.12,
                          'r_forearm_roll_joint': 1.766,
                          'r_wrist_flex_joint': -0.07,
                          'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        self.world.notify_state_change()
        location_desig = CostmapLocation(self.world.get_body_by_name("milk.stl"), visible_for=self.robot_view)
        plan = SequentialPlan(self.context, self.robot_view, NavigateActionDescription(location_desig))
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)

    def test_visibility_pose_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                         'l_shoulder_lift_joint': -0.264,
                         'l_upper_arm_roll_joint': 1.38,
                         'l_elbow_flex_joint': -2.12,
                         'l_forearm_roll_joint': 16.996,
                         'l_wrist_flex_joint': -0.073,
                         'l_wrist_roll_joint': 0.0,
                        'r_shoulder_pan_joint': -1.712,
                          'r_shoulder_lift_joint': -0.256,
                          'r_upper_arm_roll_joint': -1.463,
                          'r_elbow_flex_joint': -2.12,
                          'r_forearm_roll_joint': 1.766,
                          'r_wrist_flex_joint': -0.07,
                          'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        self.world.notify_state_change()
        location_desig = CostmapLocation(PoseStamped.from_list(self.world.root, [1, 0, 1]), visible_for=self.robot_view)
        plan = SequentialPlan(self.context, self.robot_view, NavigateActionDescription(location_desig))
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)

    def test_reachability_and_visibility_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                    'l_shoulder_lift_joint': -0.264,
                    'l_upper_arm_roll_joint': 1.38,
                    'l_elbow_flex_joint': -2.12,
                    'l_forearm_roll_joint': 16.996,
                    'l_wrist_flex_joint': -0.073,
                    'l_wrist_roll_joint': 0.0,
                    'r_shoulder_pan_joint': -1.712,
                    'r_shoulder_lift_joint': -0.256,
                    'r_upper_arm_roll_joint': -1.463,
                    'r_elbow_flex_joint': -2.12,
                    'r_forearm_roll_joint': 1.766,
                    'r_wrist_flex_joint': -0.07,
                    'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position = 0.3
        self.world.notify_state_change()
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[RobotDescription.current_robot_description.name])
        location_desig = CostmapLocation(object_desig.resolve(), reachable_for=robot_desig.resolve(),
                                         visible_for=robot_desig.resolve())
        plan = SequentialPlan(self.context, self.robot_view, NavigateActionDescription(location_desig))
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)
        #self.assertTrue(Arms.LEFT == location.reachable_arm or Arms.RIGHT == location.reachable_arm)

    def test_reachability_probabilistic_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                    'l_shoulder_lift_joint': -0.264,
                    'l_upper_arm_roll_joint': 1.38,
                    'l_elbow_flex_joint': -2.12,
                    'l_forearm_roll_joint': 16.996,
                    'l_wrist_flex_joint': -0.073,
                    'l_wrist_roll_joint': 0.0,
                    'r_shoulder_pan_joint': -1.712,
                    'r_shoulder_lift_joint': -0.256,
                    'r_upper_arm_roll_joint': -1.463,
                    'r_elbow_flex_joint': -2.12,
                    'r_forearm_roll_joint': 1.766,
                    'r_wrist_flex_joint': -0.07,
                    'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position = 0.3
        self.world.notify_state_change()
        location_desig = ProbabilisticCostmapLocation(self.world.get_body_by_name("milk.stl"), reachable_for=self.robot_view)
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)
        #self.assertTrue(Arms.LEFT == location.reachable_arm or Arms.RIGHT == location.reachable_arm)

    def test_reachability_pose_probabilistic_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                    'l_shoulder_lift_joint': -0.264,
                    'l_upper_arm_roll_joint': 1.38,
                    'l_elbow_flex_joint': -2.12,
                    'l_forearm_roll_joint': 16.996,
                    'l_wrist_flex_joint': -0.073,
                    'l_wrist_roll_joint': 0.0,
                    'r_shoulder_pan_joint': -1.712,
                    'r_shoulder_lift_joint': -0.256,
                    'r_upper_arm_roll_joint': -1.463,
                    'r_elbow_flex_joint': -2.12,
                    'r_forearm_roll_joint': 1.766,
                    'r_wrist_flex_joint': -0.07,
                    'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        location_desig = ProbabilisticCostmapLocation(PoseStamped.from_list(self.world.root, [0.4, 0.6, 0.9], [0, 0, 0, 1]), reachable_for=self.robot_view)
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)
        #self.assertTrue(Arms.LEFT == location.reachable_arm or Arms.RIGHT == location.reachable_arm)

    def test_visibility_probabilistic_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                    'l_shoulder_lift_joint': -0.264,
                    'l_upper_arm_roll_joint': 1.38,
                    'l_elbow_flex_joint': -2.12,
                    'l_forearm_roll_joint': 16.996,
                    'l_wrist_flex_joint': -0.073,
                    'l_wrist_roll_joint': 0.0,
                    'r_shoulder_pan_joint': -1.712,
                    'r_shoulder_lift_joint': -0.256,
                    'r_upper_arm_roll_joint': -1.463,
                    'r_elbow_flex_joint': -2.12,
                    'r_forearm_roll_joint': 1.766,
                    'r_wrist_flex_joint': -0.07,
                    'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        location_desig = ProbabilisticCostmapLocation(self.world.get_body_by_name("milk.stl"), visible_for=self.robot_view)
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)

    def test_visibility_pose_probabilistic_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                    'l_shoulder_lift_joint': -0.264,
                    'l_upper_arm_roll_joint': 1.38,
                    'l_elbow_flex_joint': -2.12,
                    'l_forearm_roll_joint': 16.996,
                    'l_wrist_flex_joint': -0.073,
                    'l_wrist_roll_joint': 0.0,
                    'r_shoulder_pan_joint': -1.712,
                    'r_shoulder_lift_joint': -0.256,
                    'r_upper_arm_roll_joint': -1.463,
                    'r_elbow_flex_joint': -2.12,
                    'r_forearm_roll_joint': 1.766,
                    'r_wrist_flex_joint': -0.07,
                    'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        location_desig = ProbabilisticCostmapLocation(PoseStamped.from_list(self.world.root, [1, 0, 1]), visible_for=self.robot_view)
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)

    def test_reachability_and_visibility_probabilistic_costmap_location(self):
        arm_park = {'l_shoulder_pan_joint': 1.712,
                    'l_shoulder_lift_joint': -0.264,
                    'l_upper_arm_roll_joint': 1.38,
                    'l_elbow_flex_joint': -2.12,
                    'l_forearm_roll_joint': 16.996,
                    'l_wrist_flex_joint': -0.073,
                    'l_wrist_roll_joint': 0.0,
                    'r_shoulder_pan_joint': -1.712,
                    'r_shoulder_lift_joint': -0.256,
                    'r_upper_arm_roll_joint': -1.463,
                    'r_elbow_flex_joint': -2.12,
                    'r_forearm_roll_joint': 1.766,
                    'r_wrist_flex_joint': -0.07,
                    'r_wrist_roll_joint': 0.051}
        for name, state in arm_park.items():
            self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position = state
        self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position = 0.3
        self.world.notify_state_change()
        location_desig = ProbabilisticCostmapLocation(self.world.get_body_by_name("milk.stl"), reachable_for=self.robot_view,
                                         visible_for=self.robot_view)
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)
        #self.assertTrue(Arms.LEFT == location.reachable_arm or Arms.RIGHT == location.reachable_arm)

    def test_semantic_location(self):
        location_desig = SemanticCostmapLocation(self.world.get_body_by_name("island_countertop"))
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)

        location_desig = SemanticCostmapLocation(self.world.get_body_by_name("island_countertop"),
                                                 for_object=self.world.get_body_by_name("milk.stl"))
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)

    def test_probabilistic_semantic_location(self):
        kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
        location_desig = ProbabilisticSemanticLocation(["kitchen_island_surface"], kitchen_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)

        milk_desig = ObjectDesignatorDescription(names=["milk"])
        location_desig = ProbabilisticSemanticLocation(["kitchen_island_surface"], kitchen_desig.resolve(),
                                                 for_object=milk_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()) == 3)
        self.assertTrue(len(location.orientation.to_list()) == 4)

    def test_accessing_location(self):
        location_desig = AccessingLocation(self.world.get_body_by_name("handle_cab10_t"), robot_desig=self.robot_view, arm=Arms.RIGHT)
        plan = SequentialPlan(self.context, self.robot_view, NavigateActionDescription(location_desig))
        access_pose = location_desig.resolve()

        self.assertTrue(len(access_pose.position.to_list()) == 3)
        self.assertTrue(len(access_pose.orientation.to_list()) == 4)

