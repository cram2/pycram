from pycram.designator import MotionDesignator
from pycram.bullet_world import BulletWorld


def pr2_motion_designators(desig):
    solutions = []
    if desig.check_constraints([('type', 'moving'), 'target']):
        solutions.append(desig.make_dictionary([('cmd', 'navigate'), 'target']))

    if desig.check_constraints([('type', 'pick-up'), 'object', 'target']):
        if desig.check_constraints([('arm', 'right')]):
            solutions.append(desig.make_dictionary([('cmd', 'pick'), 'object', 'target', ('gripper', 'r_gripper_tool_frame')]))
        solutions.append(desig.make_dictionary([('cmd', 'pick'), 'object', 'target', ('gripper', 'l_gripper_tool_frame')]))

    if desig.check_constraints([('type', 'place'), 'target']):
        if desig.check_constraints(['object']):
            if desig.check_constraints([('arm', 'right')]):
                solutions.append(desig.make_dictionary([('cmd', 'place'), 'target', 'object', ('gripper', 'r_gripper_tool_frame')]))
            solutions.append(desig.make_dictionary([('cmd', 'place'), 'target', 'object', ('gripper', 'l_gripper_tool_frame')]))

        if desig.check_constraints([('arm', 'right')]):
            solutions.append(desig.make_dictionary([('cmd', 'place'), 'target',
                                                   ('object', list(BulletWorld.robot.attachments.keys())[0]), ('gripper', 'r_gripper_tool_frame')]))
        solutions.append(desig.make_dictionary([('cmd', 'place'), 'target',
                                               ('object', list(BulletWorld.robot.attachments.keys())[0]), ('gripper', 'l_gripper_tool_frame')]))

    if desig.check_constraints([('type', 'accessing'), 'drawer']):
        if desig.check_constraints(['distance']):
            if desig.check_constraints([('part-of', 'kitchen')]):
                solutions.append(desig.make_dictionary(('cmd', 'access'), 'drawer', 'part-of', 'distance'))
            solutions.append(desig.make_dictionary([('cmd', 'access'), 'drawer', 'distance',
                                                    ('part-of', BulletWorld.current_bullet_world.get_object_by_name("kitchen"))]))

        if desig.check_constraints([('part-of', 'kitchen')]):
            solutions.append(desig.make_dictionary([('cmd', 'access'), 'drawer', 'part-of', ('distance', 0.3)]))
        solutions.append(desig.make_dictionary([('cmd', 'access'), 'drawer', ('distance', 0.3),
                                                ('part-of', BulletWorld.current_bullet_world.get_object_by_name("kitchen"))]))

        if desig.check_constraints([('type', 'park-arms')]):
            solutions.append(desig.make_dictonary([('cmd', 'park')]))

    return solutions


MotionDesignator.resolvers.append(pr2_motion_designators)
