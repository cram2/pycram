import time
from pycram.language import SequentialPlan

from pycram.process_module import simulated_robot

from pycram.datastructures.enums import Arms

from pycram.robot_plans import ParkArmsActionDescription

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix as tm
from semantic_digital_twin.utils import get_semantic_digital_twin_directory_root
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world_description.geometry import Box, Scale, Sphere, Cylinder, FileMesh, Color

import logging
import os
from semantic_digital_twin.utils import get_semantic_digital_twin_directory_root
from semantic_digital_twin.views import factories

sw = World()
root = Body(name= PrefixedName('root', "world"))
body_box1 = Body(
    name=PrefixedName("box1", "PhysicalObject"),
)
body_box2 = Body(
    name=PrefixedName("box2", "PhysicalObject"),
)
body_box3 = Body(
    name=PrefixedName("box3", "PhysicalObject"),
)
box1_start = (0.75, 0.00, 0.9 ,0,0,0)
box2_start = (0.75, 0.50, 0.9 ,0,0,0)
box3_start = (0.75, 0.25, 0.9 ,0,0,0)

box1_target = (0.75, 0.50, 0.10)
box2_target = (0.75, 0.50, 0.15)
box3_target = (0.75, 0.50, 0.20)

box1 = Box(tm(reference_frame=body_box1),scale=Scale(0.05,0.05,0.05), color=Color(1,0,0,1))
box2 = Box(tm(reference_frame=body_box2),scale=Scale(0.05,0.05,0.05), color=Color(0,1,0,1))
box3 = Box(tm(reference_frame=body_box3),scale=Scale(0.05,0.05,0.05), color=Color(0,0,1,1))


body_box1.collision = body_box1.visual = ShapeCollection([box1], body_box1)
body_box2.collision = body_box2.visual = ShapeCollection([box2], body_box2)
body_box3.collision = body_box3.visual = ShapeCollection([box3], body_box3)

#add boxes to world
with sw.modify_world():
    c_root_box1 = Connection6DoF(parent=root, child=body_box1, _world=sw)
    sw.add_connection(c_root_box1)

    c_root_box2 = Connection6DoF(parent=root, child=body_box2, _world=sw)
    sw.add_connection(c_root_box2)

    c_root_box3 = Connection6DoF(parent=root, child=body_box3, _world=sw)
    sw.add_connection(c_root_box3)


c_root_box1.origin = tm.from_xyz_rpy(*box1_start, reference_frame=root)
c_root_box2.origin = tm.from_xyz_rpy(*box2_start, reference_frame=root)
c_root_box3.origin = tm.from_xyz_rpy(*box3_start, reference_frame=root)


#tracy_world = URDFParser.from_file(os.path.join("/home/mrskooma/workspace/ros/src/semantic_digital_twin", "resources", "urdf", "tracy.urdf")).parse()
tracy_world = URDFParser.from_file(os.path.join(os.path.dirname(__file__), "..", "resources", "robots", "pr2.urdf")).parse()
sw.merge_world(tracy_world)
# from semantic_digital_twin.robots import Tracy
# robot_view = Tracy.from_world(sw)
#

import pycram.robot_descriptions.pr2_states
from semantic_digital_twin.robots.pr2 import PR2
robot_view = PR2.from_world(sw)

print(sw)

from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
import threading
import rclpy
#rclpy.init()

node = rclpy.create_node("semantic_digital_twin")
# thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
# thread.start()

viz = VizMarkerPublisher(world=sw, node=node)


# c_root_box1.origin = sw.transform(tm.from_xyz_rpy(x=-0.3, y=0.2, reference_frame=robot_view.left_arm.manipulator.tool_frame), target_frame=root)
# ik = sw.compute_inverse_kinematics(robot_view.root, robot_view.left_arm.tip, target=body_box1.global_pose, max_iterations=1000)
# with sw.modify_world():
#     for key, value in ik.items():
#         print(key, value)
#         sw.state[key.name].position = value

# print(ik)
# print("testing")
# time.sleep(5)
# print("ik test")
# # ik = sw.compute_inverse_kinematics(robot_view.root, robot_view.left_arm.tip, target=body_box1.global_pose, max_iterations=1000)
# # with sw.modify_world():
# #     for joint, position in ik.items():
# #         sw.state[joint.name].position = position
# time.sleep(5)
# print("plan test")
# time.sleep(5)
description = ParkArmsActionDescription([Arms.BOTH])
with simulated_robot:
    SequentialPlan((sw, None), robot_view, description).perform()

# time.sleep(10)
print("done")
node.destroy_node()
# rclpy.shutdown()
