from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.process_module import ProcessModule
from pycram.process_modules import ProcessModules
from pycram.bullet_world import BulletWorld
from pycram.helper import transform
from pycram.ik import request_ik
from pycram.helper import _transform_to_torso, _apply_ik
from pycram.local_transformer import local_transformer
import pycram.bullet_world_reasoning as btr
import pybullet as p
import numpy as np
import time


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of PR2 and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = BulletWorld.robot
    if arm == "right":
        for joint, pose in robot_description.i.get_static_joint_chain("right", "park").items():
            robot.set_joint_state(joint, pose)
    if arm == "left":
        for joint, pose in robot_description.i.get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class Pr2Navigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'navigate':
            robot = BulletWorld.robot
            robot.set_position_and_orientation(solution['target'], solution['orientation'])
            time.sleep(0.5)
            local_transformer.update_from_btr()


class Pr2PickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'pick':
            object = solution['object']
            robot = BulletWorld.robot
            target = object.get_position_and_orientation()
            target = _transform_to_torso(target, robot)
            arm = "left" if solution['gripper'] == robot_description.i.get_tool_frame("left") else "right"
            joints = robot_description.i._safely_access_chains(arm).joints
            #tip = "r_wrist_roll_link" if solution['gripper'] == "r_gripper_tool_frame" else "l_wrist_roll_link"
            inv = request_ik(robot_description.i.base_frame, solution['gripper'], target, robot, joints)
            _apply_ik(robot, inv, solution['gripper'])
            robot.attach(object, solution['gripper'])
            time.sleep(0.5)


class Pr2Place(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'place':
            object = solution['object']
            robot = BulletWorld.robot
            target = [solution['target'], [0, 0, 0, 1]]
            target = _transform_to_torso(target, robot)
            target = (target[0], [0, 0, 0, 1])
            arm = "left" if solution['gripper'] == robot_description.i.get_tool_frame("left") else "right"
            joints = robot_description.i._safely_access_chains(arm).joints
            #inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), solution['target'],
            #                               maxNumIterations=100)
            #tip = "r_wrist_roll_link" if solution['gripper'] == "r_gripper_tool_frame" else "l_wrist_roll_link"
            inv = request_ik(robot_description.i.base_frame, solution['gripper'], target, robot, joints)
            _apply_ik(robot, inv, solution['gripper'])
            robot.detach(object)
            time.sleep(0.5)


class Pr2Accessing(ProcessModule):
    """
    This process module responsible for opening drawers to access the objects inside. This works by firstly moving
    the end effector to the handle of the drawer. Next, the end effector is moved the respective distance to the back.
    This provides the illusion the robot would open the drawer by himself.
    Then the drawer will be opened by setting the joint pose of the drawer joint.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'access':
            kitchen = solution['part_of']
            robot = BulletWorld.robot
            gripper = solution['gripper']
            drawer_handle = solution['drawer_handle']
            drawer_joint = solution['drawer_joint']
            dis = solution['distance']
            arm = "left" if solution['gripper'] == robot_description.i.get_tool_frame("left") else "right"
            joints = robot_description.i._safely_access_chains(arm).joints
            #inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), kitchen.get_link_position(drawer_handle))
            target = _transform_to_torso(kitchen.get_link_position_and_orientation(drawer_handle), robot)
            target = (target[0], [0, 0, 0, 1])
            inv = request_ik(robot_description.i.base_frame, gripper, target , robot, joints )
            _apply_ik(robot, inv, gripper)
            time.sleep(0.2)
            new_p = ([target[0][0] - dis, target[0][1], target[0][2]],target[1])
            #inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), new_p)
            inv = request_ik(robot_description.i.base_frame, gripper, new_p, robot, joints)
            _apply_ik(robot, inv, gripper)
            kitchen.set_joint_state(drawer_joint, dis)
            time.sleep(0.5)


class Pr2ParkArms(ProcessModule):
    """
    This process module is for moving the arms in a parking position.
    It is currently not used.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'park':
            _park_arms()


class Pr2MoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'looking':
            target = solutions['target']
            robot = BulletWorld.robot
            if type(target) is str:
                target_frame = local_transformer.projection_namespace + '/' + target \
                    if local_transformer.projection_namespace \
                    else target
                target = local_transformer.tf_transform(local_transformer.map_frame, target_frame)[0]
            pose_in_pan = transform(target, robot.get_link_position("head_pan_link"))
            pose_in_tilt = transform(target, robot.get_link_position("head_tilt_link"))

            new_pan = np.arctan([pose_in_pan[1], pose_in_pan[0]])
            new_tilt = np.arctan([-pose_in_tilt[2], pose_in_tilt[0] ** 2 + pose_in_tilt[1] ** 2])

            robot.set_joint_state("head_pan_joint", new_pan[0])
            robot.set_joint_state("head_tilt_joint", new_tilt[0])


class Pr2MoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-gripper":
            robot = BulletWorld.robot
            gripper = solution['gripper']
            motion = solution['motion']
            for joint, state in robot_description.i.get_static_gripper_chain(gripper, motion).items():
                robot.set_joint_state(joint, state)
            time.sleep(0.5)


class Pr2Detecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "detecting":
            robot = BulletWorld.robot
            object_type = solution['object_type']
            cam_frame_name = solution['cam_frame']
            front_facing_axis = solution['front_facing_axis']

            objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)
            for obj in objects:
                if btr.visible(obj, robot.get_link_position_and_orientation(cam_frame_name), front_facing_axis):
                    return obj


class Pr2MoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-tcp":
            target = solution['target']
            target = _transform_to_torso([target, [0, 0, 0, 1]])
            gripper = solution['gripper']
            robot = BulletWorld.robot
            joints = ik_joints_left if gripper == "l_gripper_tool_frame" else ik_joints_right
            #inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), target)
            inv = request_ik(pr2_root_link, gripper, target, robot, joints)
            _apply_ik(robot, inv, gripper)
            time.sleep(0.5)


class Pr2MoveJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-joints":
            robot = BulletWorld.robot
            right_arm_poses = solution['right_arm_poses']
            left_arm_poses = solution['left_arm_poses']

            if type(right_arm_poses) == dict:
                for joint, pose in right_arm_poses.items():
                    robot.set_joint_state(joint, pose)
            elif type(right_arm_poses) == str and right_arm_poses == "park":
                _park_arms("right")

            if type(left_arm_poses) == dict:
                for joint, pose in left_arm_poses.items():
                    robot.set_joint_state(joint, pose)
            elif type(right_arm_poses) == str and left_arm_poses == "park":
                _park_arms("left")

            time.sleep(0.5)


class Pr2WorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "world-state-detecting":
            obj_type = solution['object_type']
            return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class PR2ProcessModules(ProcessModules):
    initialized = None

    def __init__(self):
        if not PR2ProcessModules.initialized:
            super().__init__(Pr2Navigation(), Pr2PickUp(), Pr2Place(), Pr2Accessing(), Pr2ParkArms(), Pr2MoveHead(),
                             Pr2MoveGripper(), Pr2MoveGripper(), Pr2Detecting(), Pr2MoveTCP(), Pr2MoveJoints(),
                             Pr2WorldStateDetecting())
            PR2ProcessModules.initialized = self
