from pycram.designator import MotionDesignator
from pycram.bullet_world import BulletWorld


def pr2_motion_designators(desig):
    solutions = []
    if desig.check_constraints([('type', 'moving'), 'target']):
        if desig.check_constraints(['orientation']):
            solutions.append(desig.make_dictionary([('cmd', 'navigate'), 'target', 'orientation']))
        solutions.append(desig.make_dictionary([('cmd', 'navigate'), 'target', ('orientation', BulletWorld.robot.get_orientation())]))

    if desig.check_constraints([('type', 'pick-up'), 'object']):
        if desig.check_constraints([('arm', 'right')]):
            solutions.append(desig.make_dictionary([('cmd', 'pick'), 'object', ('gripper', 'r_gripper_tool_frame')]))
        solutions.append(desig.make_dictionary([('cmd', 'pick'), 'object', ('gripper', 'l_gripper_tool_frame')]))

    if desig.check_constraints([('type', 'place'), 'target']):
        if desig.check_constraints(['object']):
            if desig.check_constraints([('arm', 'right')]):
                solutions.append(desig.make_dictionary([('cmd', 'place'), 'target', 'object', ('gripper', 'r_gripper_tool_frame')]))
            solutions.append(desig.make_dictionary([('cmd', 'place'), 'target', 'object', ('gripper', 'l_gripper_tool_frame')]))

    if desig.check_constraints([('type', 'accessing'), 'drawer-joint', 'drawer-handle', 'part-of']):
        if desig.check_constraints([('arm', 'right')]):
            if desig.check_constraints('distance'):
                solutions.append(desig.make_dictionary([('cmd', 'access'), 'drawer-joint', 'drawer-handle', ('gripper', 'r_gripper_tool_frame'), 'distance', 'part-of']))
            solutions.append(desig.make_dictionary([('cmd', 'access'), 'drawer-joint', 'drawer-handle', ('gripper', 'r_gripper_tool_frame'), ('distance', 0.3), 'part-of']))
        solutions.append(desig.make_dictionary([('cmd', 'access'), 'drawer-joint', 'drawer-handle', 'part-of', ('distance', 0.3), ('gripper', 'l_gripper_tool_frame'), 'part-of']))

    if desig.check_constraints([('type', 'move-tcp'), 'target']):
        if desig.check_constraints([('arm', 'right')]):
            solutions.append(desig.make_dictionary([('cmd', 'move-tcp'), 'target', ('gripper', 'r_gripper_tool_frame')]))
        solutions.append(desig.make_dictionary([('cmd', 'move-tcp'), 'target', ('gripper', 'l_gripper_tool_frame')]))

    if desig.check_constraints([('type', 'park-arms')]):
        solutions.append(desig.make_dictionary([('cmd', 'park')]))

    if desig.check_constraints([('type', 'looking')]):
        if desig.check_constraints(['target']):
            solutions.append(desig.make_dictionary([('cmd', 'looking'), 'target']))
        if desig.check_constraints(['object']):
            solutions.append(desig.make_dictionary([('cmd', 'looking'), ('target', BulletWorld.current_bullet_world.
                                                                         get_objects_by_name(desig.prop_value('object')).get_pose())]))

    if desig.check_constraints([('type', 'opening-gripper'), 'gripper']):
        solutions.append(desig.make_dictionary([('cmd', 'move-gripper'), ('motion', 'open'), 'gripper']))

    if desig.check_constraints([('type', 'closing-gripper'), 'gripper']):
        solutions.append(desig.make_dictionary([('cmd', 'move-gripper'), ('motion', 'close'), 'gripper']))

    if desig.check_constraints([('type', 'detecting'), 'object']):
        solutions.append(desig.make_dictionary([('cmd', 'detecting'), ('cam_frame', 'wide_stereo_optical_frame'), 'object']))

    if desig.check_constraints([('type', 'move-tcp'), 'target']):
        if desig.check_constraints([('arm', 'right')]):
            solutions.append(desig.make_dictionary([('cmd', 'move-tcp'), 'arm', 'target']))
        solutions.append(desig.make_dictionary([('cmd', 'move-tcp'), ('arm', 'left'), 'target']))

    if desig.check_constraints([('type', 'move-arm-joints')]):
        if desig.check_constraints(['left-arm', 'right-arm']):
            solutions.append(desig.make_dictionary([('cmd', 'move-joints'), ('left-poses', desig.prop_value('left-arm')), ('right-poses', desig.prop_value('right-arm'))]))
        if desig.check_constraints(['left-arm']):
            solutions.append(desig.make_dictionary([('cmd', 'move-joints'), ('left-poses', desig.prop_value('left-arm')), ('right-poses', None)]))
        if desig.check_constraints(['right-arm']):
            solutions.append(desig.make_dictionary([('cmd', 'move-joints'), ('right-poses', desig.prop_value('right-arm')), ('left-poses', None)]))

    if desig.check_constraints([('type', 'world-state-detecting')]):
        solutions.append(desig.make_dictionary([('cmd', 'world-state-detecting'), 'object']))

    return solutions


MotionDesignator.resolvers.append(pr2_motion_designators)
