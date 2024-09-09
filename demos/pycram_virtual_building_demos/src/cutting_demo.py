from pycram.datastructures.dataclasses import Color
from pycram.process_module import simulated_robot
from pycram.designators.action_designator import *
from pycram.datastructures.enums import Arms, TorsoState
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
from pycram.world_concepts.world_object import Object


def start_cutting(obj, technique):
    try:
        _technique = technique.split(":", 1)[1]
    except IndexError:
        _technique = technique
    #######################################################################################

    drawer_island_surface = Pose([2.4, 2, 0.95], [0, 0, -1, -1])
    right_to_sink_surface = Pose([2.4, 2.3, 0.95], [0, 0, -1, -1])
    drawer_island_surface_board = Pose([2.45, 2, 0.95], [0, 0, -1, -1])
    pose = drawer_island_surface

    board = Object("board", ObjectType.BOARD, "board.stl", pose=pose)
    color = Color(0.4, 0.2, 0.06, 1)
    obj_tool = Object("knife", ObjectType.CUTTING_TOOL, "butter_knife.stl", pose=pose)
    board.set_color(color)
    objects = [(None, None), ('apple', "obo:FOODON_03301710"), ('avocado', "obo:FOODON_00003600"),
               ('banana', "obo:FOODON_00004183"), ('citron', "obo:FOODON_03306596"),
               ('cucumber', "obo:FOODON_00003415"),
               ('kiwi', "obo:FOODON_00004387"), ('lemon', "obo:FOODON_03301441"), ('lime', "obo:FOODON_00003661"),
               ('orange', "obo:FOODON_03309832"), ('peach', "obo:FOODON_03315502"), ('tomato', "obo:FOODON_03309927")]

    for id_, name in objects:
        if name == obj or id_ == obj:
            obj_path = id_ + ".stl"
            obj = id_

    obj_to_cut = Object(obj, ObjectType.GENERIC_OBJECT, path=obj_path, pose=pose)

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        location_pose = Pose([1.7, 2, 0])
        looking_pose = Pose([2.5, 2, 0.97])
        NavigateAction([location_pose]).resolve().perform()
        if RobotDescription.current_robot_description.name == "pr2":
            MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
        if RobotDescription.current_robot_description.name == "Armar6":
            tool_pose = Pose([2.2049586673391935, 1.40084467778416917, 1.0229705326966067],
                                 [0, 0, 1, 1])
        # else is pr2
        else:
            tool_pose = Pose([2.0449586673391935, 1.5384467778416917, 1.229705326966067],
                                 [0.14010099565491793, -0.7025332835765593, 0.15537176280408957, 0.6802046102510538])
        obj_tool.pose = tool_pose

        tool_frame = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()
        World.current_world.robot.attach(child_object=obj_tool, parent_link=tool_frame)
        if RobotDescription.current_robot_description.name == "Armar6":
            MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.RIGHT).perform()
            location_pose = Pose([1.6, 2.5, 0])
            NavigateAction([location_pose]).resolve().perform()

        LookAtAction([looking_pose]).resolve().perform()
        generic_obj_BO = BelieveObject(names=[obj]).resolve()

        bigknife_BO = BelieveObject(names=["knife"]).resolve()
        CuttingAction(generic_obj_BO, bigknife_BO, [Arms.RIGHT], _technique).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        obj_to_cut.remove()
        World.current_world.remove_vis_axis()

#

# start_cutting("obo:FOODON_03301710", "soma:Slicing")
# start_cutting("obo:FOODON_03301710", "cut:Halving")
#
# #avocado
# start_cutting("obo:FOODON_00003600", "soma:Cutting")
# start_cutting("obo:FOODON_00003600", "soma:Slicing")
# start_cutting("obo:FOODON_00003600", "cut:Halving")
#
# #banana
# start_cutting("obo:FOODON_00004183", "soma:Cutting")
# start_cutting("obo:FOODON_00004183", "soma:Slicing")
# start_cutting("obo:FOODON_00004183", "cut:Halving")
#
#
# #citron
# start_cutting("obo:FOODON_03306596", "soma:Cutting")
# start_cutting("obo:FOODON_03306596", "soma:Slicing")
# start_cutting("obo:FOODON_03306596", "cut:Halving")


# print("cut")
# start_cutting("obo:FOODON_00003415", "soma:Cutting")
# print("slice")
# start_cutting("obo:FOODON_00003415", "soma:Slicing")
# print("halving")
# start_cutting("obo:FOODON_00003415", "cut:Halving")

# #kiwi
# start_cutting("obo:FOODON_00004387", "soma:Cutting")
# start_cutting("obo:FOODON_00004387", "soma:Slicing")
# start_cutting("obo:FOODON_00004387", "cut:Halving")

#
# #lemon
# start_cutting("obo:FOODON_03301441", "soma:Cutting")
# start_cutting("obo:FOODON_03301441", "soma:Slicing")
# start_cutting("obo:FOODON_03301441", "cut:Halving")
#
# #lime
# start_cutting("obo:FOODON_00003661", "soma:Cutting")
# start_cutting("obo:FOODON_00003661", "soma:Slicing")
# start_cutting("obo:FOODON_00003661", "cut:Halving")
#
# #orange
# start_cutting("obo:FOODON_03309832", "soma:Cutting")
# start_cutting("obo:FOODON_03309832", "soma:Slicing")
# start_cutting("obo:FOODON_03309832", "cut:Halving")
#
#
# #peach
# start_cutting("obo:FOODON_03315502", "soma:Cutting")
# start_cutting("obo:FOODON_03315502", "soma:Slicing")
# start_cutting("obo:FOODON_03315502", "cut:Halving")


# #peach
# start_cutting("obo:FOODON_03309927", "soma:Cutting")
# start_cutting("obo:FOODON_03309927", "soma:Slicing")
# start_cutting("obo:FOODON_03309927", "cut:Halving")


# [INFO] [1710184677.654084]: Querying the ontology for the cutting task
# [INFO] [1710184680.265410]: The repetition for the task is: 0.05
# [INFO] [1710184680.405835]: The start position is: http://www.ease-crc.org/ont/food_cutting#halving_position
# [INFO] [1710184680.543266]: The tool to cut with is:  http://www.ease-crc.org/ont/food_cutting#ParingKnife
# [INFO] [1710184680.547092]: Queried all necessary information from the ontology
# [INFO] [1710184688.615853]: Cutting task completed!
