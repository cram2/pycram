from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.process_module import ProcessModule
from pycram.process_modules import ProcessModules
from pycram.bullet_world import BulletWorld
from pycram.local_transformer import local_transformer
import pycram.helper as helper
import pycram.bullet_world_reasoning as btr
import pybullet as p
from rospy import logerr
import time


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arm of Donbot and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = BulletWorld.robot
    if arm == "left":
        for joint, pose in robot_description.i.get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class DonbotNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'navigate':
            robot = BulletWorld.robot
            # Reset odom joints to zero
            for joint_name in robot_description.i.odom_joints:
                robot.set_joint_state(joint_name, 0.0)
            # Set actual goal pose
            robot.set_position_and_orientation(solution['target'], solution['orientation'])
            time.sleep(0.5)
            local_transformer.update_from_btr()

class DonbotPickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'pick':
            object = solution['object']
            robot = BulletWorld.robot
            target = object.get_position()
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), target,
                                               maxNumIterations=100)
            helper._apply_ik(robot, inv)
            robot.attach(object, solution['gripper'])
            time.sleep(0.5)


class DonbotPlace(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'place':
            object = solution['object']
            robot = BulletWorld.robot
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), solution['target'],
                                               maxNumIterations=100)
            helper._apply_ik(robot, inv)
            robot.detach(object)
            time.sleep(0.5)


class DonbotAccessing(ProcessModule):
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
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper),
                                               kitchen.get_link_position(drawer_handle))
            helper._apply_ik(robot, inv)
            time.sleep(0.2)
            han_pose = kitchen.get_link_position(drawer_handle)
            new_p = [han_pose[0] - dis, han_pose[1], han_pose[2]]
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), new_p)
            helper._apply_ik(robot, inv)
            kitchen.set_joint_state(drawer_joint, 0.3)
            time.sleep(0.5)


class DonbotParkArms(ProcessModule):
    """
    This process module is for moving the arms in a parking position.
    It is currently not used.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'park':
            _park_arms()


class DonbotMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'looking':
            robot = BulletWorld.robot
            neck_base_frame = local_transformer.projection_namespace + '/' + robot_description.i.chains["neck"].base_link if \
                local_transformer.projection_namespace else \
                robot_description.i.chains["neck"].base_link
            if type(solutions['target']) is str:
                target = local_transformer.projection_namespace + '/' + solutions['target'] if \
                    local_transformer.projection_namespace else \
                    solutions['target']
                pose_in_neck_base = local_transformer.tf_transform(neck_base_frame, target)
            elif helper.is_list_pose(solutions['target']) or helper.is_list_position(solutions['target']):
                pose = helper.ensure_pose(solutions['target'])
                pose_in_neck_base = local_transformer.tf_pose_transform(local_transformer.map_frame, neck_base_frame, pose)

            vector = pose_in_neck_base[0]
            # +x as forward
            # +y as left
            # +z as up
            x = vector[0]
            y = vector[1]
            z = vector[2]
            conf = None
            if y > 0:
                conf = "left"
            else:
                conf = "right"
            for joint, state in robot_description.i.get_static_joint_chain("neck", conf).items():
                robot.set_joint_state(joint, state)


class DonbotMoveGripper(ProcessModule):
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
                # TODO: Test this, add gripper-opening/-closing to the demo.py
                robot.set_joint_state(joint, state)
            time.sleep(0.5)


class DonbotDetecting(ProcessModule):
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
                if btr.visible(obj, robot.get_link_position_and_orientation(cam_frame_name), front_facing_axis, 0.5):
                    return obj


class DonbotMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-tcp":
            target = solution['target']
            gripper = solution['gripper']
            robot = BulletWorld.robot
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), target)
            helper._apply_ik(robot, inv)
            time.sleep(0.5)


class DonbotMoveJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-joints":
            robot = BulletWorld.robot
            left_arm_poses = solution['left_poses']

            if type(left_arm_poses) == dict:
                for joint, pose in left_arm_poses.items():
                    robot.set_joint_state(joint, pose)
            elif type(left_arm_poses) == str and left_arm_poses == "park":
                _park_arms("left")

            time.sleep(0.5)


class DonbotWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "world-state-detecting":
            obj_type = solution['object_type']
            return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class DonbotProcessModules(ProcessModules):
    initialized = None

    def __init__(self):
        if not DonbotProcessModules.initialized:
            super().__init__(DonbotNavigation(), DonbotPickUp(), DonbotPlace(), DonbotAccessing(), DonbotParkArms(),
                             DonbotMoveHead(), DonbotMoveGripper(), DonbotMoveGripper(), DonbotDetecting(),
                             DonbotMoveTCP(), DonbotMoveJoints(), DonbotWorldStateDetecting())
            DonbotProcessModules.initialized = self
