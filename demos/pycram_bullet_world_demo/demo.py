import os

from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.reasoning.world_reasoner import WorldReasoner
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.semantic_annotations.semantic_annotations import Container
from semantic_digital_twin.adapters.procthor.procthor_semantic_annotations import Milk, Bowl, Spoon
from semantic_digital_twin.spatial_types import TransformationMatrix
from semantic_digital_twin.world_description.connections import FixedConnection

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TorsoState, Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, TransportActionDescription
from pycram.robot_plans import ParkArmsActionDescription
from pycram.testing import setup_world

world = setup_world()

spoon = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "spoon.stl")).parse()
bowl = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "bowl.stl")).parse()

with world.modify_world():
    world.merge_world_at_pose(bowl, TransformationMatrix.from_xyz_quaternion(2.4, 2.2, 1, reference_frame=world.root))
    connection = FixedConnection(parent=world.get_body_by_name("cabinet10_drawer_top"), child=spoon.root)
    world.merge_world(spoon, connection)

try:
    import rclpy

    rclpy.init()
    from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher

    v = VizMarkerPublisher(world, rclpy.create_node("viz_marker"))
except ImportError:
    pass

pr2 = PR2.from_world(world)
context = Context.from_world(world)


with world.modify_world():
    world_reasoner = WorldReasoner(world)
    world_reasoner.reason()
    world.add_semantic_annotations([Bowl(body=world.get_body_by_name("bowl.stl")),
                                    Spoon(body=world.get_body_by_name("spoon.stl"))
                                    ])

plan = SequentialPlan(context,
                      ParkArmsActionDescription(Arms.BOTH),
                      MoveTorsoActionDescription(TorsoState.HIGH),
                      TransportActionDescription(world.get_body_by_name("milk.stl"),
                                                 PoseStamped.from_list([4.9, 3.3, 0.8], frame=world.root), Arms.LEFT),
                      TransportActionDescription(world.get_body_by_name("spoon.stl"),
                                                 PoseStamped.from_list([5.1, 3.3, 0.75], [0, 0, 1, 1], frame=world.root), Arms.LEFT),
                      TransportActionDescription(world.get_body_by_name("bowl.stl"),
                                                 PoseStamped.from_list([5, 3.3, 0.75], frame=world.root), Arms.LEFT))

with simulated_robot:
    plan.perform()
