from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld
import pycram.bullet_world_reasoning as btr
import pybullet as p


def _apply_ik(robot, joint_poses):
    for i in range(0, p.getNumJoints(robot.id)):
        qIndex = p.getJointInfo(robot.id, i)[3]
        if qIndex > -1:
            p.resetJointState(robot.id, i, joint_poses[qIndex-7])


def _park_arms():
    joint_poses = [] # TODO add park joint poses
    robot = BulletWorld.robot
    for i in range(0, p.getNumJoints(robot.id)):
        qIndex = p.getJointInfo(robot.id, i)[3]
        if qIndex > -1:
            p.resetJointState(robot.id, i, joint_poses[qIndex-7])


class Pr2Navigation(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'move':
            robot = BulletWorld.current_bullet_world.get_object_by_id(solution['id'])
            pre_pose = robot.get_pose()
            robot.set_position(solution['pose'])
            for obj in BulletWorld.current_bullet_world.objects:
                if btr.contact(robot, obj):
                    robot.set_position(pre_pose)
                    raise IOError # TODO fix error


class Pr2PickUp(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'pick':
            object = BulletWorld.current_bullet_world.get_object_by_id(solution['id'])
            robot = BulletWorld.robot
            if not btr.reachable(object, BulletWorld.robot, solution['gripper']):
                raise IOError # TODO fix error
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), solution['target'],
                                               maxNumIterations=100)
            _apply_ik(robot, inv)
            robot.attach(object, solution['gripper'])
            _park_arms()


class Pr2Place(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'pick':
            object = BulletWorld.current_bullet_world.get_object_by_id(solution['id'])
            robot = BulletWorld.robot
            if not btr.reachable(object, robot, solution['gripper']):
                raise IOError # TODO fix error
        inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), solution['target'],
                                           maxNumIterations=100)
        _apply_ik(robot, inv)
        robot.detach(object)
        _park_arms()


class Pr2Accessing(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'access':
            kitchen = BulletWorld.current_bullet_world.get_object_by_id(solution['id'])
            robot = BulletWorld.robot
            robot.attach(BulletWorld.current_bullet_world.get_object_by_id(solution['drawer']))
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), solution['target'],
                                               maxNumIteratoins=100)
            _apply_ik(robot, inv)
            p.resetJointState(kitchen, solution['link'], solution['target'])
            robot.detach(BulletWorld.current_bullet_world.get_object_by_id(solution['drawer']))


pr2_navigation = Pr2Navigation()
pr2_pick_up = Pr2PickUp()
pr2_place = Pr2Place()
pr2_accessing = Pr2Accessing()


def available_process_modules(desig):
    if desig.check_constraints([('type', 'moving')]):
        return pr2_navigation

    if desig.check_constraints([('type', 'pick-up')]):
        return pr2_pick_up

    if desig.check_constraints([('type', 'place')]):
        return pr2_place

    if desig.check_constraints([('type', 'accessing')]):
        return pr2_accessing


ProcessModule.resolvers.append(available_process_modules)


