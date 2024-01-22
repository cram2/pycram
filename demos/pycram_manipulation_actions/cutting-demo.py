import subprocess

import rospy

from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
from pycram.resolver.action.cutting import CuttingActionSPARQL
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()
world.set_gravity([0, 0, -9.8])
robot = Object("pr2", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
#kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
spawning_poses = {
    'bigknife': Pose([0.9, 0.6, 0.8], object_orientation),
    'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, -1]),
    'board': Pose([-0.85, 0.9, 0.85], [0, 0, -1, -1]),
    'cocumber': Pose([-0.85, 0.9, 0.87], [0, 0, -1, -1])
}
bigknife = Object("bigknife", "bigknife", "big-knife.stl", spawning_poses["bigknife"])
cocumber = Object("cocumber", "cocumber", "cocumber.stl", spawning_poses["cocumber"])
board = Object("board", "board", "board.stl", spawning_poses["board"])
cocumber.set_color([0, 1, 0.04, 1])
board.set_color([0.4, 0.2, 0.06, 1])
bigknife_BO = BelieveObject(names=["bigknife"])
bread_BO = BelieveObject(names=["bread"])
cocumber_BO = BelieveObject(names=["cocumber"])


with simulated_robot:
    output = subprocess.check_output(["rospack", "find", "pycram"], universal_newlines=True).strip()
    print(output)
    pickup_knife_pose = Pose([0.76, -0.08, 0], [0, 0, 0.6318018199283353, 0.7751299635127281])
    cutting_pose = Pose([-0.3, 0.9, 0.0], [0, 0, 1, 6.123233995736766e-17])
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.33]).resolve().perform()
    grasp = robot_description.grasps.get_orientation_for_grasp("top")
    arm = "left"
    pickup_pose_knife = CostmapLocation(target=bigknife_BO.resolve(), reachable_for=robot_desig).resolve()
    pickup_arm = pickup_pose_knife.reachable_arms[0]
    NavigateAction(target_locations=[pickup_knife_pose]).resolve().perform()
    PickUpAction(object_designator_description=bigknife_BO,
                 arms=["left"],
                 grasps=["top"]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    original_quaternion = (0, 0, 0, 1)
    rotation_axis = (0, 0, 1)
    rotation_quaternion = helper.axis_angle_to_quaternion(rotation_axis, 180)
    resulting_quaternion = helper.multiply_quaternions(original_quaternion, rotation_quaternion)

    nav_pose = Pose([-0.3, 0.9, 0.0], resulting_quaternion)

    NavigateAction(target_locations=[cutting_pose]).resolve().perform()
    LookAtAction(targets=[cocumber_BO.resolve().pose]).resolve().perform()

    detected_bread_desig = DetectAction(cocumber_BO).resolve().perform()

    CuttingAction(cocumber_BO,bigknife_BO,["left"], "slicing").resolve().perform()
    CuttingActionSPARQL(cocumber_BO, bigknife_BO, ["left"]).resolve().perform()
    rospy.loginfo("Cutting done")
