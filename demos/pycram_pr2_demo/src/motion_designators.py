from pycram.designator import MotionDesignator

def pr2_motion_designators(desig):
	solutions = []

	if desig.check_constraints([('type', 'moving'), 'pose']):
		if desig.check_constraints(['wait']):
			if desig.check_constraints(['frame']):
				solutions.append(desig.make_dictionary([('cmd', 'move'), 'pose', 'wait', 'frame']))

			solutions.append(desig.make_dictionary([('cmd', 'move'), 'pose', 'wait', ('frame', 'base_footprint')]))

		if desig.check_constraints(['frame']):
			solutions.append(desig.make_dictionary([('cmd', 'move'), 'pose', ('wait', True), 'frame']))

		solutions.append(desig.make_dictionary([('cmd', 'move'), 'pose', ('wait', True), ('frame', 'base_footprint')]))

	if desig.check_constraints([('type', 'pointing_head'), 'coordinates']):
		if desig.check_constraints(['wait']):
			if desig.check_constraints(['frame']):
				solutions.append(desig.make_dictionary([('cmd', 'point_head'), 'coordinates', 'wait', 'frame']))

			solutions.append(desig.make_dictionary([('cmd', 'point_head'), 'coordinates', 'wait', ('frame', 'base_footprint')]))

		if desig.check_constraints(['frame']):
			solutions.append(desig.make_dictionary([('cmd', 'point_head'), 'coordinates', ('wait', True), 'frame']))

		solutions.append(desig.make_dictionary([('cmd', 'point_head'), 'coordinates', ('wait', True), ('frame', 'base_footprint')]))

	if desig.check_constraints([('type', 'setting_arm_joints'), 'positions']):
		if desig.check_constraints(['wait']):
			solutions.append(desig.make_dictionary([('cmd', 'set_arm_joints'), 'positions', 'wait']))

		solutions.append(desig.make_dictionary([('cmd', 'set_arm_joints'), 'positions', ('wait', True)]))

	if desig.check_constraints([('type', 'setting_gripper'), 'position']):
		if desig.check_constraints(['wait']):
			if desig.check_constraints(['max_effort']):
				solutions.append(desig.make_dictionary([('cmd', 'set_gripper'), 'position', 'wait', 'max_effort']))

			solutions.append(desig.make_dictionary([('cmd', 'set_gripper'), 'position', 'wait', ('max_effort', 25)]))

		if desig.check_constraints(['max_effort']):
			solutions.append(desig.make_dictionary([('cmd', 'set_gripper'), 'position', ('wait', True), 'max_effort']))

		solutions.append(desig.make_dictionary([('cmd', 'set_gripper'), 'position', ('wait', True), ('max_effort', 25)]))

	if desig.check_constraints([('type', 'setting_torso'), 'position']):
		if desig.check_constraints(['wait']):
			solutions.append(desig.make_dictionary([('cmd', 'set_torso'), 'position', 'wait']))

		solutions.append(desig.make_dictionary([('cmd', 'set_torso'), 'position', ('wait', True)]))

	if desig.check_constraints([('type', 'subscribing_pose'), 'fl']):
		solutions.append(desig.make_dictionary([('cmd', 'subscribe_pose'), 'fl']))

	if desig.check_constraints([('type', 'subscribing_gripper'), 'fl']):
		if desig.check_constraints(['gripper']):
			solutions.append(desig.make_dictionary([('cmd', 'subscribe_gripper'), 'gripper', 'fl']))

	if desig.check_constraints([('type', 'cancelling_movement')]):
		solutions.append(desig.make_dictionary([('cmd', 'cancel_movement')]))

	return solutions

MotionDesignator.resolvers.append(pr2_motion_designators)
