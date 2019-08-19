from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld
import pycram.bullet_world_reasoning as btr
import pybullet as p
import time


def _apply_ik(robot, joint_poses):
    for i in range(0, p.getNumJoints(robot.id)):
        qIndex = p.getJointInfo(robot.id, i)[3]
        if qIndex > -1:
            p.resetJointState(robot.id, i, joint_poses[qIndex-7])


def _park_arms():
    joint_poses = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.9, -0.1, 1.6, 1.7,
                   0.087, 1.2, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9, -0.1, 1.6,
                   -1.7, -0.08, -1.2, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    _apply_ik(BulletWorld.robot, joint_poses)


class Pr2Navigation(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'navigate':
            robot = BulletWorld.robot
            pre_pose = robot.get_pose()
            robot.set_position(solution['target'])
            #for obj in BulletWorld.current_bullet_world.objects:
            #    if btr.contact(robot, obj):
            #        robot.set_position(pre_pose)
            #        raise IOError # TODO fix error


class Pr2PickUp(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'pick':
            object = solution['object']
            robot = BulletWorld.robot
            if not btr.reachable(object, BulletWorld.robot, solution['gripper']):
                raise IOError # TODO fix error
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), solution['target'],
                                               maxNumIterations=100)
            _apply_ik(robot, inv)
            robot.attach(object, solution['gripper'])
            _park_arms()
            pos = p.getLinkState(robot.id, robot.get_link_id(solution['gripper']))[0]
            p.resetBasePositionAndOrientation(object.id, pos, object.get_orientation())
            BulletWorld.current_bullet_world.simulate(1)
            time.sleep(0.5)


class Pr2Place(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'place':
            object = solution['object']
            robot = BulletWorld.robot
            if not btr.reachable(object, robot, solution['gripper']):
                raise IOError # TODO fix error
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), solution['target'],
                                           maxNumIterations=100)
            _apply_ik(robot, inv)
            p.resetBasePositionAndOrientation(object.id, solution['target'], object.get_orientation())
            robot.detach(object)
            _park_arms()
            time.sleep(0.5)


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


