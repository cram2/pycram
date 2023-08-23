from pycram.designators.action_designator import PickUpAction, PlaceAction, ParkArmsAction, MoveTorsoAction, \
    NavigateAction, PourAction
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot
from pycram.enums import Arms
from pycram.pose import Pose
# from pycram.designator import ObjectDesignatorDescription as odd
from pycram.bullet_world import BulletWorld, Object
from pycram.designators.location_designator import CostmapLocation, SemanticCostmapLocation
import tf


world = BulletWorld()
world.set_gravity([0, 0, -9.8])

# source container
milk = Object("milk", "milk", "milk.stl", pose=Pose([2.4, 2.5, 1]))
# destination container
bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([2.4, 2.8, 0.98]))
pr2 = Object("pr2", "robot", "pr2.urdf", pose=Pose([1.2, 2.5, 0]))

pr2_desig = BelieveObject(names=["pr2"])
milk_desig = BelieveObject(names=["milk"])
bowl_desig = BelieveObject(names=["bowl"])

apartment = Object("apartment", "environment", "apartment.urdf")

with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.3]).resolve().perform()

    # find pick up pose for source container
    pickup_pose = CostmapLocation(target=milk_desig.resolve(), reachable_for=pr2_desig.resolve()).resolve()
    pickup_arm = pickup_pose.reachable_arms[0]

    # navigate to the source container
    NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    # perform pickup action
    PickUpAction(object_designator_description=milk_desig, arms=[pickup_arm], grasps=["front"]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
    # calculate pouring angle in quaternion
    quaternion = tf.transformations.quaternion_from_euler(90, 0, 0, axes="sxyz")
    # for simplicity calculate tilting pose where the destination container is. In this case a bowl
    tilting_pose = SemanticCostmapLocation.Location(pose=Pose( bowl.original_pose.position_as_list(), list(quaternion)))
    revert_tilting_pose = SemanticCostmapLocation.Location(pose=Pose(bowl.original_pose.position_as_list(), [0.0, 0, 0, 1]))
    # do pouring by tilting, and accept time interval
    PourAction(milk_desig, pouring_location=[tilting_pose.pose], revert_location=[revert_tilting_pose.pose],
               arms=[pickup_arm], wait_duration=5).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
    # place the source container to its original position
    place_pose = SemanticCostmapLocation.Location(pose=Pose(milk.original_pose.position_as_list(), [0.0, 0, 0, 1.0]))
    PlaceAction(milk_desig, target_locations=[place_pose.pose], arms=[pickup_arm]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
    # move pr2 to end pose
    end_pose = SemanticCostmapLocation.Location(pose=Pose([1, 2.5, 0], [0.0, 0, 0, 1.0]))
    NavigateAction(target_locations=[end_pose.pose]).resolve().perform()
