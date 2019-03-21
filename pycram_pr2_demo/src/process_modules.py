import control as c

from pycram.process_module import ProcessModule

class Pr2Navigation(ProcessModule):
	def _execute(self, designator):
		solution = designator.reference()

		if solution['cmd'] == 'move':
			c.move_to(solution['pose'], solution['wait'], solution['frame'])

class Pr2Head(ProcessModule):
	def _execute(self, designator):
		solution = designator.reference()

		if solution['cmd'] == 'point_head':
			coordinates = solution['coordinates']
			c.point_head_to(coordinates[0], coordinates[1], coordinates[2], solution['wait'], solution['frame'])

class Pr2ArmL(ProcessModule):
	def _execute(self, designator):
		solution = designator.reference()

		if solution['cmd'] == 'set_arm_joints':
			c.set_arm_joints('l', solution['positions'], solution['wait'])

class Pr2ArmR(ProcessModule):
	def _execute(self, designator):
		solution = designator.reference()

		if solution['cmd'] == 'set_arm_joints':
			c.set_arm_joints('r', solution['positions'], solution['wait'])

class Pr2GripperL(ProcessModule):
	def _execute(self, designator):
		solution = designator.reference()

		if solution['cmd'] == 'set_gripper':
			c.set_gripper('l', solution['position'], solution['wait'], solution['max_effort'])

class Pr2GripperR(ProcessModule):
	def _execute(self, designator):
		solution = designator.reference()

		if solution['cmd'] == 'set_gripper':
			c.set_gripper('r', solution['position'], solution['wait'], solution['max_effort'])

class Pr2Torso(ProcessModule):
	def _execute(self, designator):
		solution = designator.reference()

		if solution['cmd'] == 'set_torso':
			c.set_torso(solution['position'], solution['wait'])

class Pr2Subscriber(ProcessModule):
	def _execute(self, designator):
		solution = designator.reference()

		if solution['cmd'] == 'subscribe_pose':
			c.subscribe_pose(solution['fl'])
		elif solution['cmd'] == 'subscribe_gripper':
			c.subscribe_gripper(solution['gripper'], solution['fl'])

class Pr2NavigationCancelling(ProcessModule):
	def _execute(self, designator):
		solution = designator.reference()

		if solution['cmd'] == 'cancel_movement':
			c.cancel_movement()

pr2_navigation = Pr2Navigation()
pr2_head = Pr2Head()
pr2_arm_l = Pr2ArmL()
pr2_arm_r = Pr2ArmR()
pr2_gripper_l = Pr2GripperL()
pr2_gripper_r = Pr2GripperR()
pr2_torso = Pr2Torso()
pr2_subscriber = Pr2Subscriber()
pr2_navigation_cancelling = Pr2NavigationCancelling()

def available_pr2_process_modules(desig):
	if desig.check_constraints([('type', 'moving')]):
		return pr2_navigation

	if desig.check_constraints([('type', 'pointing_head')]):
		return pr2_head

	if desig.check_constraints([('type', 'setting_arm_joints')]):
		if desig.check_constraints([('arm', '0')]):
			return pr2_arm_l

		if desig.check_constraints([('arm', '1')]):
			return pr2_arm_r

	if desig.check_constraints([('type', 'setting_gripper')]):
		if desig.check_constraints([('gripper', '0')]):
			return pr2_gripper_l

		if desig.check_constraints([('gripper', '1')]):
			return pr2_gripper_r

	if desig.check_constraints([('type', 'setting_torso')]):
		return pr2_torso

	if desig.check_constraints([('type', 'subscribing_pose')]) or desig.check_constraints([('type', 'subscribing_gripper')]):
		return pr2_subscriber

	if desig.check_constraints([('type', 'cancelling_movement')]):
		return pr2_navigation_cancelling

ProcessModule.resolvers.append(available_pr2_process_modules)
