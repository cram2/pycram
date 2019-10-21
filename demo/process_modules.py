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
            robot.set_position(solution['target'])
            for obj in BulletWorld.current_bullet_world.objects:
                if btr.contact(robot, obj):
                    if obj.name == "floor":
                        continue
                    raise IOError # TODO fix error



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
            time.sleep(0.3)
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


class Pr2ParkArms(ProcessModule):
    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'park':
            _park_arms()


class Pr2MoveHead(ProcessModule):
    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'looking':
            target = solutions['target']
            joint_id = solutions['joint-id']
            robot = BulletWorld.robot


class Pr2MoveGripper(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-gripper":
            robot = BulletWorld.robot
            left_conf = solution['left']
            right_conf = solution['right']
            if right_conf:
                p.resetJointState(robot.id, 57, 0 if right_conf == "close" else 0.548)
                p.resetJointState(robot.id, 59, 0 if right_conf == "close" else 0.548)
            if left_conf:
                p.resetJointState(robot.id, 79, 0 if right_conf == "close" else 0.548)
                p.resetJointState(robot.id, 81, 0 if right_conf == "close" else 0.548)


class Pr2Detecting(ProcessModule):
    def _execute(self, desig):
        solultion = desig.reference()
        if solultion['cmd'] == "detecting":
            robot = BulletWorld.robot
            object_type = solultion['object']
            cam_frame_id = solultion['cam_frame']

            objects = BulletWorld.current_bullet_world.objects
            visible_objects = []
            for obj in objects:
                if obj.type == "environment":
                    continue
                if btr.visible(obj, BulletWorld.current_bullet_world.get_object_by_id(cam_frame_id).get_pose()):
                    visible_objects.append(obj)

            for obj in visible_objects:
                if obj.type == object_type:
                    return obj


class Pr2MoveTCP(ProcessModule):
    def _execute(self, desig):
        return None


class Pr2MoveJoints(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-joints":
            robot = BulletWorld.robot
            right_arm_poses = solution['right-poses']
            left_arm_poses = solution['left-poses']
            if right_arm_poses:
                for i in range(0, 9):
                    p.resetJointState(robot.id, i + 42, right_arm_poses[i])

            if left_arm_poses:
                for i in range(0, 9):
                    p.resetJointState(robot.id, i + 64, left_arm_poses[i])


# Maybe Implement
class Pr2WorldStateDetecting(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "world-state-detecting":
            obj_type = solution['object']
            return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


pr2_navigation = Pr2Navigation()
pr2_pick_up = Pr2PickUp()
pr2_place = Pr2Place()
pr2_accessing = Pr2Accessing()
pr2_park_arms = Pr2ParkArms()
pr2_move_head = Pr2MoveHead()
pr2_move_gripper = Pr2MoveGripper()
pr2_detecting = Pr2Detecting()
pr2_move_tcp = Pr2MoveTCP()
pr2_move_joints = Pr2MoveJoints()
pr2_world_state_detecting = Pr2WorldStateDetecting()


def available_process_modules(desig):
    if desig.check_constraints([('type', 'moving')]):
        return pr2_navigation

    if desig.check_constraints([('type', 'pick-up')]):
        return pr2_pick_up

    if desig.check_constraints([('type', 'place')]):
        return pr2_place

    if desig.check_constraints([('type', 'accessing')]):
        return pr2_accessing

    if desig.check_constraints([('type', 'park-arms')]):
        return pr2_park_arms


ProcessModule.resolvers.append(available_process_modules)


