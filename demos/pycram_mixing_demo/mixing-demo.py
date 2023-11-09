from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper


world = BulletWorld()
world.set_gravity([0, 0, -9.8])
# plane = Object("floor", "environment", "plane.urdf", world=world)
robot = Object("pr2", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

spawning_poses = {
    'whisk': Pose([0.9, 0.6, 0.8], [0, 0, 0, -1]),
    'big-bowl': Pose([-0.85, 0.9, 1], [0, 0, -1, -1])
}
whisk = Object("whisk", "whisk", "whisk.stl", spawning_poses["whisk"])
big_bowl = Object("big-bowl", "big-bowl", "big-bowl.stl", spawning_poses["big-bowl"])
whisk_BO = BelieveObject(names=["whisk"])
big_bowl_BO = BelieveObject(names=["big-bowl"])


with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    MoveTorsoAction([0.33]).resolve().perform()
    grasp = robot_description.grasps.get_orientation_for_grasp("top")
    arm = "left"
    #
    pickup_pose_knife = CostmapLocation(target=whisk_BO.resolve(), reachable_for=robot_desig).resolve()
    pickup_arm = pickup_pose_knife.reachable_arms[0]

    NavigateAction(target_locations=[pickup_pose_knife.pose]).resolve().perform()

    PickUpAction(object_designator_description=whisk_BO,
                 arms=["left"],
                 grasps=["top"]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
    original_quaternion = (0, 0, 0, 1)
    rotation_axis = (0, 0, 1)

    rotation_quaternion = helper.axis_angle_to_quaternion(rotation_axis, 180)
    resulting_quaternion = helper.multiply_quaternions(original_quaternion, rotation_quaternion)

    nav_pose = Pose([-0.3, 0.9, 0.0], resulting_quaternion)

    # NavigateAction(target_locations=[pickup_pose_knife.pose]).resolve().perform()
    NavigateAction(target_locations=[nav_pose]).resolve().perform()
    LookAtAction(targets=[big_bowl_BO.resolve().pose]).resolve().perform()

    MixingAction(object_designator_description=big_bowl_BO,
                 object_tool_designator_description=big_bowl_BO,
                 arms=["left"],
                 grasps=["top"]).resolve().perform()
