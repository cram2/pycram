from pycram.datastructures.dataclasses import Color
from pycram.process_module import simulated_robot
from pycram.designators.action_designator import *
from pycram.datastructures.enums import Arms, TorsoState
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
from pycram.world_concepts.world_object import Object


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

    pose = Pose([2.4, 2, 0.95], [0, 0, -1, -1])

    if type == "mixing":
        objects_tool = [(None, None), ('whisk', "WHISKKNOWLEDGEBASE")]
        objects_target = [(None, None), ('big-bowl', "BOWLKNOWLEDBASE")]
        obj_tool_ = create_object(obj_tool, ObjectType.MIXING_TOOL, objects_tool, pose)
        obj_target_ = create_object(obj_target, ObjectType.GENERIC_OBJECT, objects_target, pose)

    # cutting
    else:
        objects_target = [(None, None), ('apple', "obo:FOODON_03301710"), ('avocado', "obo:FOODON_00003600"),
                          ('banana', "obo:FOODON_00004183"), ('citron', "obo:FOODON_03306596"),
                          ('cucumber', "obo:FOODON_00003415"), ('kiwi', "obo:FOODON_00004387"),
                          ('lemon', "obo:FOODON_03301441"), ('lime', "obo:FOODON_00003661"),
                          ('orange', "obo:FOODON_03309832"), ('peach', "obo:FOODON_03315502"),
                          ('tomato', "obo:FOODON_03309927")]
        obj_tool_ = Object("knife", ObjectType.CUTTING_TOOL, "butter_knife.stl", pose)
        obj_target_ = create_object(obj_target, ObjectType.GENERIC_OBJECT, objects_target, pose)

    action_map = {"cutting": CuttingAction, "mixing": MixingAction}

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

        obj_tool_.pose = tool_pose

        tool_frame = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()
        World.current_world.robot.attach(child_object=obj_tool_, parent_link=tool_frame)
        if RobotDescription.current_robot_description.name == "Armar6":
            MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.RIGHT).perform()
            location_pose = Pose([1.6, 2.5, 0])
            NavigateAction([location_pose]).resolve().perform()

        LookAtAction([looking_pose]).resolve().perform()

        generic_obj_BO = BelieveObject(names=[obj_target_.name]).resolve()
        tool_BO = BelieveObject(names=[obj_tool_.name]).resolve()

        action = action_map[type]
        action(generic_obj_BO, tool_BO, [Arms.RIGHT], _technique).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        obj_tool_.remove()
        obj_target_.remove()
        World.current_world.remove_vis_axis()
