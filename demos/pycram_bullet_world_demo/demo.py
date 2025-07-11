from pycram.language import SequentialPlan, ParallelPlan, CodePlan
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.process_module import simulated_robot, with_simulated_robot, ProcessModule
from pycram.object_descriptors.urdf import ObjectDescription as URDFObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap.ontologies import Robot, Apartment, Milk, Cereal, Spoon, Bowl
import numpy as np
from pycram.robot_plans import *

np.random.seed(420)
extension = URDFObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.GUI)
viz = VizMarkerPublisher()

robot = Object("pr2", Robot, f"pr2{extension}", pose=PoseStamped.from_list([1, 2, 0]))
apartment = Object("apartment", Apartment, f"apartment{extension}")

milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([2.5, 2, 1.02], [0, 0, 0, 1]),
              color=Color(1, 0, 0, 1))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                pose=PoseStamped.from_list([2.45, 2.4, 1.05], [0, 0, 0, 1]), color=Color(0, 1, 0, 1))
spoon = Object("spoon", Spoon, "spoon.stl", pose=PoseStamped.from_list([2.4, 2.24, 0.85]),
               color=Color(0, 0, 1, 1))
bowl = Object("bowl", Bowl, "bowl.stl", pose=PoseStamped.from_list([2.35, 2.2, 0.98]),
              color=Color(1, 1, 0, 1))
apartment.attach(spoon, 'cabinet10_drawer_top')

with simulated_robot:
    sp = SequentialPlan(
        ParallelPlan(MoveTorsoActionDescription(TorsoState.HIGH),
                     ParkArmsActionDescription(Arms.BOTH)),

        TransportActionDescription(ResolutionStrategyObject(
            strategy=SearchActionDescription(PoseStamped.from_list([2.4, 1.5, 1]), Milk)),
            PoseStamped.from_list([4.8, 3.55, 0.8]), Arms.LEFT),
        TransportActionDescription(ResolutionStrategyObject(
            strategy=SearchActionDescription(PoseStamped.from_list([2.4, 1.5, 1]), Cereal)),
            PoseStamped.from_list([5.2, 3.4, 0.8], [0, 0, 1, 1]), Arms.LEFT),
        TransportActionDescription(ResolutionStrategyObject(
            strategy=SearchActionDescription(PoseStamped.from_list([2.4, 1.5, 1]), Bowl)),
            PoseStamped.from_list([5, 3.3, 0.8], [0, 0, 1, 1]), Arms.LEFT))

    sp.perform()



viz._stop_publishing()
world.exit()