from pycram.designators import MotionDesignators
from pycram.bullet_world import BulletWorld

def pr2_motion_designators(desig):
    solutions = []

    if desig.check_constraints([('type', 'moving'), 'pose']):
        solutions.append(desig.make_dictonary([('cmd', 'navigate'), ('target', 'pose')]))

    if desig.check_constraints([('type', 'pick-up'), 'object', 'target']):
        if desig.check_constraints([('arm', 'right')]):
            solutions.append(desig.make_dictonary([('cmd', 'pick'), 'object', 'target', ('gripper', 'r_gripper_tool_frame')]))
        solutions.append(desig.make_dictonary([('cmd', 'pick'), 'object', 'target', ('gripper', 'l_gripper_tool_frame')]))

    if desig.check_constraints([('type', 'place'), 'target']):
        if desig.check_constraints(['object']):
            if desig.check_constraints([('arm', 'right')]):
                solutions.append(desig.make_dictonary([('cmd', 'place'), 'target', 'object', ('gripper', 'r_gripper_tool_frame')]))
            solutions.append(desig.make_dictonary([('cmd', 'place'), 'target', 'object', ('gripper', 'l_gripper_tool_frame')]))

        if desig.check_constraints([('arm', 'right')]):
            solutions.append(desig.make_dictonary([('cmd', 'place'), 'target',
                                                   ('object', next(iter(BulletWorld.robot.attachments.values())).id), ('gripper', 'r_gripper_tool_frame')]))
        solutions.append(desig.make_dictonary([('cmd', 'place'), 'target',
                                               ('object', next(iter(BulletWorld.robot.attachments.values())).id), ('gripper', 'l_gripper_tool_frame')]))

    if desig.check_constraints([('type', 'accessing'), 'drawer']):
        if desig.check_constraints(['distance']):
            if desig.check_constraints([('part-of', 'kitchen')]):
                solutions.append(desig.make_dictonary(('cmd', 'access'), 'drawer', 'part-of', 'distance'))
            solutions.append(desig.make_dictonary([('cmd', 'access'), 'drawer', 'distance',
                                                    ('part-of', BulletWorld.current_bullet_world.get_object_by_name("kitchen"))]))

        if desig.check_constraints([('part-of', 'kitchen')]):
            solutions.append(desig.make_dictonary([('cmd', 'access'), 'drawer', 'part-of', ('distance', 0.3)]))
        solutions.append(desig.make_dictonary([('cmd', 'access'), 'drawer', ('distance', 0.3),
                                                ('part-of', BulletWorld.current_bullet_world.get_object_by_name("kitchen"))]))


MotionDesignators.resolvers.append(pr2_motion_designators)
