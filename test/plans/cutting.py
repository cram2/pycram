from pycram.process_module import simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject


def cutting_action_test():
    """
    This function is a test for the cutting action.
    """
    world = BulletWorld("DIRECT")
    robot = Object("pr2", "robot", "../../resources/" + robot_description.name + ".urdf")
    robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
    robot.set_joint_state(robot_description.torso_joint, 0.24)

    spawning_poses = {
        'bigknife': Pose([0.9, 0.6, 0.8], [0, 0, 0, -1]),
        'board': Pose([-0.85, 0.9, 0.85], [0, 0, -1, -1]),
        'cocumber': Pose([-0.85, 0.9, 0.87], [0, 0, -1, -1])
    }
    bigknife = Object("bigknife", "bigknife", "big-knife.stl", spawning_poses["bigknife"])
    cocumber = Object("cocumber", "cocumber", "cocumber.stl", spawning_poses["cocumber"])
    board = Object("board", "board", "board.stl", spawning_poses["board"])
    bigknife_BO = BelieveObject(names=["bigknife"])
    cocumber_BO = BelieveObject(names=["cocumber"])

    with simulated_robot:
        pickup_knife_pose = Pose([0.76, -0.08, 0], [0, 0, 0.6318018199283353, 0.7751299635127281])
        cutting_pose = Pose([-0.3, 0.9, 0.0], [0, 0, 1, 6.123233995736766e-17])
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        MoveTorsoAction([0.33]).resolve().perform()
        NavigateAction(target_locations=[pickup_knife_pose]).resolve().perform()
        PickUpAction(object_designator_description=bigknife_BO,
                     arms=["left"],
                     grasps=["top"]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        NavigateAction(target_locations=[cutting_pose]).resolve().perform()
        CuttingAction(cocumber_BO, bigknife_BO, ["left"], ["slicing"]).resolve().perform()


if __name__ == "__main__":
    cutting_action()
