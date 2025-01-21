from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap import Robot, Apartment, MixingTool, Genobj, Board
from pycram.helper import perform, an
np.random.seed(420)
extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)


robot = Object("pr2", Robot, f"pr2{extension}", pose=Pose([1, 2, 0]))
apartment = Object("apartment", Apartment, f"apartment{extension}")

VizMarkerPublisher()
def create_object(obj, obj_type, objects, pose):
    for id_, name in objects:
        if name == obj or id_ == obj:
            obj_path = id_ + ".stl"
            obj = id_
    return Object(obj, obj_type, path=obj_path, pose=pose)


def start_generalized_demo(type, obj_tool, obj_target, technique):
    try:
        _technique = technique.split(":", 1)[1]
    except IndexError:
        _technique = technique


    if type == "mixing":
        pose = Pose([2.4, 2, 1.0], [0, 0, -1, -1])

        objects_tool = [(None, None), ('whisk', "WHISKKNOWLEDGEBASE")]
        objects_target = [(None, None), ('big-bowl', "BOWLKNOWLEDBASE")]
        obj_tool_ = create_object(obj_tool, MixingTool, objects_tool, pose=pose)
        obj_target_ = create_object(obj_target, Genobj, objects_target, pose=pose)

    elif type == "pouring":
        pose = Pose([2.4, 2, 1.0], [0, 0, 0, 1])

        objects_tool = [(None, None), ('jeroen_cup', "WHISKKNOWLEDGEBASE")]
        objects_target = [(None, None), ('bowl', "BOWLKNOWLEDBASE")]
        obj_tool_ = create_object(obj_tool, ObjectType.POURING_TOOL, objects_tool, pose=pose)
        obj_target_ = create_object(obj_target, ObjectType.GENERIC_OBJECT, objects_target, pose=pose)
    # cutting
    else:
        pose = Pose([2.3, 2, 1.0], [0, 0, -1, -1])
        objects_tool = [(None, None), ('butter_knife', "KNIFEKNOWLEDGEBASE")]
        objects_target = [(None, None), ('apple', "obo:FOODON_03301710"), ('avocado', "obo:FOODON_00003600"),
                          ('banana', "obo:FOODON_00004183"), ('citron', "obo:FOODON_03306596"),
                          ('cucumber', "obo:FOODON_00003415"), ('kiwi', "obo:FOODON_00004387"),
                          ('lemon', "obo:FOODON_03301441"), ('lime', "obo:FOODON_00003661"),
                          ('orange', "obo:FOODON_03309832"), ('peach', "obo:FOODON_03315502"),
                          ('tomato', "obo:FOODON_03309927")]
        drawer_island_surface_board = Pose([2.45, 1.8, 0.95], [0, 0, -1, -1])
        drawer_island_surface = Pose([2.4, 1.8, 1.0], [0, 0, -1, -1])
        board = Object("board", Board, "board.stl", pose=drawer_island_surface_board)
        color = Color(0.4, 0.2, 0.06, 1)
        board.set_color(color)
        obj_tool_ = create_object(obj_tool, ObjectType.CUTTING_TOOL, objects_tool, pose=pose)
        obj_target_ = create_object(obj_target, ObjectType.GENERIC_OBJECT, objects_target, pose=drawer_island_surface)

    # this is just bc we dont want to pick up for the demonstration of cutting and mixing
    if RobotDescription.current_robot_description.name == "Armar6":
        tool_pose = Pose([2.2049586673391935, 1.40084467778416917, 1.0229705326966067],
                         [0, 0, 1, 1])
    # else is pr2
    else:
        if type in ["cutting"]:
            tool_pose = Pose([2.0449586673391935, 1.5384467778416917, 1.229705326966067],
                             [0.14010099565491793, -0.7025332835765593, 0.15537176280408957, 0.6802046102510538])
        elif type == "pouring":
            tool_pose = Pose([2.0449586673391935, 1.5384467778416917, 1.09705326966067],
                             [0, 0, 0, 1])
        elif type == "mixing":
            tool_pose = Pose([2.0449586673391935, 1.5384467778416917, 1.09705326966067],
                             [0, 1, 0, 1])

    obj_tool_.pose = tool_pose

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        location_pose = Pose([1.7, 2, 0])
        looking_pose = Pose([2.5, 2, 0.97])
        NavigateAction([location_pose]).resolve().perform()
        if RobotDescription.current_robot_description.name == "pr2":
            MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

        tool_frame = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()
        World.current_world.robot.attach(child_object=obj_tool_, parent_link=tool_frame)
        LookAtAction([looking_pose]).resolve().perform()

        generic_obj_BO = BelieveObject(names=[obj_target_.name]).resolve()
        tool_BO = BelieveObject(names=[obj_tool_.name]).resolve()

        perform(an(ActionCore(action_type="mixing", arms=[Arms.RIGHT], source_object=generic_obj_BO,
                                           technique=_technique, tool_object=tool_BO)))

        perform(an(ParkArmsAction([Arms.BOTH])))
        ParkArmsAction([Arms.BOTH]).resolve().perform()

start_generalized_demo("mixing", "whisk", "big-bowl", "spiral")

#     mixing_action = SemanticActionCore(action_type="mixing", arms=[Arms.LEFT], source_object=milk_desig.resolve(),
#                                        technique="spiral")
#     performable_mixing = mixing_action.resolve().perform()
#     # print(performable_mixing)
#     #
#     # # Invalid Action
#     # try:
#     #     invalid_action = SemanticActionCore(action_type="flying",  arms=[Arms.LEFT], object=milk_desig)
#     #     invalid_action.ground()
#     # except ValueError as e:
#     #     print(e)
