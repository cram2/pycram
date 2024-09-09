from pycram.datastructures.dataclasses import Color
from pycram.process_module import simulated_robot
from pycram.designators.action_designator import *
from pycram.datastructures.enums import Arms, TorsoState
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
from pycram.world_concepts.world_object import Object


def start_mixing(obj_tool, obj_target, technique):
    try:
        _technique = technique.split(":", 1)[1]
    except IndexError:
        _technique = technique
    #######################################################################################
    drawer_island_surface = Pose([2.4, 2, 0.95], [0, 0, -1, -1])
    right_to_sink_surface = Pose([2.4, 2.3, 0.95], [0, 0, -1, -1])
    drawer_island_surface_board = Pose([2.45, 2, 0.95], [0, 0, -1, -1])
    pose = drawer_island_surface




    objects = [(None, None), ('whisk', "WHISKKNOWLEDGEBASE")]

    for id_, name in objects:
        if name == obj or id_ == obj:
            obj_path = id_ + ".stl"
            obj = id_
    obj_tool = Object(obj, ObjectType.MIXING_TOOL, path=obj_path, pose=pose)

    objects = [(None, None), ('big-bowl', "BOWLKNOWLEDBASE")]

    for id_, name in objects:
        if name == obj or id_ == obj:
            obj_path = id_ + ".stl"
            obj = id_
    obj_target = Object(obj, ObjectType.MIXING_TOOL, path=obj_path, pose=pose)

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

        generic_obj_BO = BelieveObject(names=[obj_target.name]).resolve()
        tool_BO = BelieveObject(names=[obj_tool.name]).resolve()

        CuttingAction(generic_obj_BO, tool_BO, [Arms.RIGHT], _technique).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        obj_tool.remove()
        obj_target.remove()
        World.current_world.remove_vis_axis()


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
