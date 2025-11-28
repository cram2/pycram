import os

import rospkg

from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import WorldMode
from pycram.robot_plans import *
from pycram.external_interfaces import giskard
from pycram.helper import perform, an
from pycram.perception import detect
from pycram.process_module import real_robot
from pycram.ros_utils.robot_state_updater import WorldStateUpdater
from pycram.ros_utils.viz_marker_publisher import (
    VizMarkerPublisher,
    ManualMarkerPublisher,
)
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import PouringTool
from pycrap.ontologies import Robot, Environment, Bowl

# from pycram.perception import detect
from pycram.helper import perform, an
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.robot_plans import *
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.process_module import simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap.ontologies import (
    Robot,
    Apartment,
    Milk,
    Cereal,
    Spoon,
    Bowl,
    PouringTool,
    Apple,
    Food,
)

extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.GUI)
robot = Object("pr2", Robot, f"pr2{extension}", pose=PoseStamped.from_list([1, 2, 0]))
apartment = Object("apartment", Apartment, f"apartment{extension}")
robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])
VizMarkerPublisher()
# WorldStateUpdater("/tf", "/joint_states")

obj_tool_ = Object(
    "jeroen_cup",
    PouringTool,
    "jeroen_cup.stl",
    pose=PoseStamped.from_list([3.4, 2, 1.0], [0, 0, 0, 1]),
)
m = ManualMarkerPublisher()


def look_at():
    looking_pose = PoseStamped.from_list([2.3, 2, 0.97])
    with real_robot:
        perform(an(LookAtActionDescription(looking_pose)))


def move_the_base():
    location_pose = PoseStamped.from_list([1.7, 2, 0])
    with real_robot:
        perform(an(NavigateActionDescription(location_pose)))


def move_in_simulation():
    location_pose = PoseStamped.from_list([1.7, 2, 0])
    with simulated_robot:
        perform(an(NavigateActionDescription(location_pose)))


def cutting_simulation():
    tool_pose = PoseStamped.from_list(
        [2.0449586673391935, 1.5384467778416917, 1.229705326966067],
        [
            0.14010099565491793,
            -0.7025332835765593,
            0.15537176280408957,
            0.6802046102510538,
        ],
    )

    with simulated_robot:
        perform(an(ParkArmsActionDescription([Arms.BOTH])))
        perform(an(MoveTorsoActionDescription([TorsoState.HIGH])))
        move_in_simulation()
        knife = Object(
            "knife", PouringTool, "big-knife.stl", pose=tool_pose, size=[0.2, 0.1, 0.05]
        )
        rotate_q = utils.axis_angle_to_quaternion([0, 0, 1], 180)
        tool_frame = RobotDescription.current_robot_description.get_arm_chain(
            Arms.RIGHT
        ).get_tool_frame()
        World.current_world.robot.attach(child_object=obj_tool_, parent_link=tool_frame)
        bread1_ = Object(
            "bread1",
            Food,
            "bread.stl",
            pose=PoseStamped.from_list([2.4, 2, 1.0], [0, 0, -1, -1]),
            color=Color.from_list([1, 1, 0, 1]),
            size=[0.1, 0.2, 0.1],
        )
        perform(an(CuttingActionDescription(bread1_, knife, [Arms.RIGHT])))


def spawn_world():
    # das sollte eign innerhalb den sync modules geupdated werden

    giskard.spawn_environment()
    giskard.initial_adding_objects()


def perception():
    with real_robot:
        perform(an(DetectActionDescription(technique=DetectionTechnique.ALL)))


def park():
    with real_robot:
        perform(an(ParkArmsActionDescription([Arms.BOTH])))


spawn_world()


def pick_up(arm, object_designator):
    with real_robot:
        manu = ManualMarkerPublisher()

        robot_desig_resolved = BelieveObject(
            names=[RobotDescription.current_robot_description.name]
        ).resolve()
        # ParkArmsAction(Arms.BOTH).perform()
        pickup_loc = CostmapLocation(
            target=object_designator,
            reachable_for=robot_desig_resolved,
            reachable_arm=[arm],
        )
        # Tries to find a pick-up position for the robot that uses the given arm
        pickup_pose = pickup_loc.resolve()
        manu.publish(pickup_pose)
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {object_designator} with arm: {arm}"
            )

        NavigateAction(pickup_pose, True).perform()
        PickUpAction(
            object_designator,
            pickup_pose.arm,
            grasp_description=pickup_pose.grasp_description,
        ).perform()


def pouring():
    tool_pose = PoseStamped.from_list(
        [2.0449586673391935, 1.5384467778416917, 1.09705326966067], [0, 0, 0, 1]
    )
    obj_tool_.pose = tool_pose
    location_pose = PoseStamped.from_list([1.7, 2, 0])
    looking_pose = PoseStamped.from_list([2.3, 2, 0.97])
    # generic_obj_BO = BelieveObject(names=[obj_target_.name]).resolve()
    tool_BO = BelieveObject(names=[obj_tool_.name]).resolve()

    with real_robot:
        print("parking arms")
        perform(an(ParkArmsActionDescription([Arms.BOTH])))
        print("Navigating")
        perform(an(NavigateActionDescription([location_pose])))
        print("MoveTorso")
        perform(an(MoveTorsoActionDescription([TorsoState.HIGH])))

        # attach tool to robot
        # tool_frame = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()
        # World.current_world.robot.attach(child_object=obj_tool_, parent_link=tool_frame)
        print("Looking")
        perform(an(LookAtActionDescription([looking_pose])))
        print("Detect")
        perceived_objects = detect(BelieveObject(types=[Bowl]))
        # query(PouringActionDescription)
        print("Pouring")
        perform(
            an(
                PouringActionDescription(
                    perceived_objects[0], tool_BO, [Arms.RIGHT], None, angle=90
                )
            )
        )
        print("ParkArms")
        perform(an(ParkArmsActionDescription([Arms.BOTH])))


def cutting():
    # Object("apple", Apple, stl_path, pose=PoseStamped.from_list([2.5, 2, 1.0],
    #                                                             [0, 0, 0, 1]),
    #        color=Color.from_list([1, 0, 0, 0, 1]))
    Object(
        "bread",
        Food,
        "bread.stl",
        pose=PoseStamped.from_list([2.5, 2, 1.0], [0, 0, 0, 1]),
        color=Color.from_list([1, 0, 0, 0, 1]),
    )
