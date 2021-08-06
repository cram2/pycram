from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.process_module import ProcessModule
from pycram.process_modules import ProcessModules
from pycram.bullet_world import BulletWorld
from pycram.helper import _apply_ik
import pycram.bullet_world_reasoning as btr
import pybullet as p
from rospy import logerr
import time


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arm of HSR and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = BulletWorld.robot
    if arm == "left":
        for joint, pose in robot_description.i.get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class HSRNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'navigate':
            robot = BulletWorld.robot
            robot.set_position_and_orientation(solution['target'], solution['orientation'])


class HSRPickUp(ProcessModule):
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
            _apply_ik(robot, inv)
            robot.attach(object, solution['gripper'])
            time.sleep(0.5)


class HSRPlace(ProcessModule):
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
            _apply_ik(robot, inv)
            robot.detach(object)
            time.sleep(0.5)


class HSRAccessing(ProcessModule):
    """
    This process module responsible for opening drawers to access the objects inside. This works by firstly moving
    the end effector to the handle of the drawer. Next, the end effector is moved the respective distance to the back.
    This provides the illusion the robot would open the drawer by himself.
    Then the drawer will be opened by setting the joint pose of the drawer joint.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'access':
            kitchen = solution['part-of']
            robot = BulletWorld.robot
            gripper = solution['gripper']
            drawer_handle = solution['drawer-handle']
            drawer_joint = solution['drawer-joint']
            dis = solution['distance']
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper),
                                               kitchen.get_link_position(drawer_handle))
            _apply_ik(robot, inv)
            time.sleep(0.2)
            han_pose = kitchen.get_link_position(drawer_handle)
            new_p = [han_pose[0] - dis, han_pose[1], han_pose[2]]
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), new_p)
            _apply_ik(robot, inv)
            kitchen.set_joint_state(drawer_joint, 0.3)
            time.sleep(0.5)


class HSRParkArms(ProcessModule):
    """
    This process module is for moving the arms in a parking position.
    It is currently not used.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'park':
            _park_arms()


class HSRMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'looking':
            target = solutions['target']
            if target is 'forward' or target is 'down':
                robot = BulletWorld.robot
                for joint, state in robot_description.i.get_static_joint_chain("neck", target).items():
                    robot.set_joint_state(joint, state)
            else:
                logerr("There is no target position defined with the target %s.", target)


class HSRMoveGripper(ProcessModule):
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


class HSRDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "detecting":
            robot = BulletWorld.robot
            object_type = solution['object']
            cam_frame_name = solution['cam_frame']
            front_facing_axis = solution['front_facing_axis']

            objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)
            for obj in objects:
                if btr.visible(obj, robot.get_link_position_and_orientation(cam_frame_name), front_facing_axis, 0.5):
                    return obj


class HSRMoveTCP(ProcessModule):
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
            _apply_ik(robot, inv)
            time.sleep(0.5)


class HSRMoveJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-joints":
            robot = BulletWorld.robot
            left_arm_poses = solution['left-poses']

            if type(left_arm_poses) == dict:
                for joint, pose in left_arm_poses.items():
                    robot.set_joint_state(joint, pose)
            elif type(left_arm_poses) == str and left_arm_poses == "park":
                _park_arms("left")

            time.sleep(0.5)


class HSRWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "world-state-detecting":
            obj_type = solution['object']
            return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class HSRProcessModules(ProcessModules):
    initialized = None

    def __init__(self):
        if not HSRProcessModules.initialized:
            super().__init__(HSRNavigation(), HSRPickUp(), HSRPlace(), HSRAccessing(), HSRParkArms(),
                             HSRMoveHead(), HSRMoveGripper(), HSRMoveGripper(), HSRDetecting(),
                             HSRMoveTCP(), HSRMoveJoints(), HSRWorldStateDetecting())
            HSRProcessModules.initialized = self
