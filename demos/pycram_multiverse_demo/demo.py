import pycrap
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, Arms, DetectionTechnique, VerticalAlignment, \
    ApproachDirection
from pycram.datastructures.pose import PoseStamped
from pycram.robot_plans import *
from pycram.designators.location_designator import CostmapLocation, AccessingLocation
from pycram.designators.object_designator import BelieveObject, ObjectPart
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.process_module import simulated_robot
from pycram.robot_description import RobotDescription
from pycram.world_concepts.world_object import Object
from pycram.worlds.multiverse import Multiverse

world = Multiverse()
extension = ObjectDescription.get_file_extension()
robot = Object('pr2', pycrap.Robot, f'pr2{extension}', pose=PoseStamped.from_list([1.3, 2, 0.01]))
apartment = Object("apartment", pycrap.Apartment, f"apartment{extension}")

milk = Object("milk", pycrap.Milk, f"milk.xml", pose=PoseStamped.from_list([2.4, 2, 1.02]),
              color=Color(1, 0, 0, 1))

spoon = Object("spoon", pycrap.Spoon, "spoon.xml", pose=PoseStamped.from_list([2.5, 2.2, 0.85]),
               color=Color(0, 0, 1, 1))
apartment.attach(spoon, 'cabinet10_drawer1')

robot_desig = BelieveObject(names=[robot.name])
apartment_desig = BelieveObject(names=[apartment.name])

with simulated_robot:

    # Transport the milk
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.25]).resolve().perform()

    NavigateAction(target_locations=[PoseStamped.from_list([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[PoseStamped.from_list([2.6, 2.15, 1])]).resolve().perform()

    milk_desig = DetectAction(DetectionTechnique.TYPES,
                              object_designator_description=BelieveObject(types=[pycrap.Milk])).resolve().perform()[0]

    TransportAction(milk_desig, [PoseStamped.from_list([2.4, 3, 1.02])], [Arms.LEFT]).resolve().perform()

    # Find and navigate to the drawer containing the spoon
    handle_desig = ObjectPart(names=["cabinet10_drawer1_handle"], part_of=apartment_desig.resolve())
    drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                        robot_desig=robot_desig.resolve()).resolve()

    NavigateAction([drawer_open_loc.pose]).resolve().perform()

    OpenAction(object_designator_description=handle_desig,
               arms=[drawer_open_loc.arm]).resolve().perform()

    arm_ee = RobotDescription.current_robot_description.get_arm_chain(drawer_open_loc.arm).get_tool_frame()
    closing_arm_pose = robot.get_link_pose(arm_ee)

    # Detect and pickup the spoon
    spoon.detach(apartment)
    LookAtAction([apartment.get_link_pose("cabinet10_drawer1_handle")]).resolve().perform()

    spoon_desig = DetectAction(DetectionTechnique.TYPES,
                               object_designator_description=BelieveObject(types=[pycrap.Spoon])).resolve().perform()[0]

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    pickup_arm = Arms.LEFT if drawer_open_loc.arm == Arms.RIGHT else Arms.RIGHT
    pick_up_loc = CostmapLocation(target=spoon_desig.pose, reachable_for=robot_desig.resolve(),
                                  reachable_arm=pickup_arm, grasp_descriptions=GraspDescription(ApproachDirection.FRONT, VerticalAlignment.TOP, False)).resolve()

    NavigateAction([pick_up_loc.pose]).resolve().perform()
    MoveTCPMotion(closing_arm_pose, drawer_open_loc.arm).perform()

    PickUpAction(spoon_desig, [pickup_arm], GraspDescription(ApproachDirection.FRONT, VerticalAlignment.TOP, False)).resolve().perform()

    ParkArmsAction([Arms.LEFT if pickup_arm == Arms.LEFT else Arms.RIGHT]).resolve().perform()

    NavigateAction([drawer_open_loc.pose]).resolve().perform()

    CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arm]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.15]).resolve().perform()

    # Find a pose to place the spoon, move and then place it
    spoon_target_pose = PoseStamped.from_list([2.35, 2.6, 0.95], [0, 0, 0, 1])
    placing_loc = CostmapLocation(target=spoon_target_pose,
                                  reachable_for=robot_desig.resolve()).resolve()

    NavigateAction([placing_loc.pose]).resolve().perform()

    PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

world.exit()


def debug_place_spoon():
    robot.set_pose(PoseStamped.from_list([1.66, 2.56, 0.01], [0.0, 0.0, -0.04044101807153309, 0.9991819274072855]))
    spoon.set_pose(PoseStamped.from_list([1.9601757566599975, 2.06718019258626, 1.0427727691273496],
                               [0.11157527804553227, -0.7076243776942466, 0.12773644958149588, 0.685931554070963]))
    robot.attach(spoon, 'r_gripper_tool_frame')
    pickup_arm = Arms.RIGHT
    spoon_desig = BelieveObject(names=[spoon.name])
    return pickup_arm, spoon_desig
