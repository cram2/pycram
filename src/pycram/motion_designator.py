from .designator import Designator, DesignatorError, ResolutionError, DesignatorDescription
from .bullet_world import Object
from .process_module import ProcessModule


class MotionDesignator(Designator):
	"""
	Implementation of motion designators.
	"""

	def __init__(self, description, parent=None):
		super().__init__(description, parent)

	def next_solution(self):
		try:
			self.reference()
		except DesignatorError:
			pass

		if self._solutions.has(self._index + 1):
			desig = MotionDesignator(self._description, self)
			desig._solutions = self._solutions
			desig._index = self._index + 1
			return desig

		return None

	def ground(self):
		return None

	def get_slots(self):
		return self._description.get_slots()

	def __str__(self):
		return "MotionDesignator({})".format(self._description.__dict__)

	def perform(self):
		return ProcessModule.perform(self)


class MotionDesignatorDescription(DesignatorDescription):
	cmd: str

	def __init__(self, resolver="grounding"):
		super().__init__(resolver)

	def get_slots(self):
		"""
		Returns a list of all slots of this description. Can be used for inspecting
		different descriptions and debugging.
		:return: A list of all slots.
		"""
		return list(self.__dict__.keys()).remove('cmd')


class MoveMotionDescription(MotionDesignatorDescription):
	"""
	Definition of types. Is used in _check_missing_properties for evaluating
	the types of given properties.
	"""
	target: list
	orientation: list
	def __init__(self, target, orientation=None, resolver="grounding"):
		super().__init__(resolver)
		self.cmd = "navigate"
		self.target = target
		self.orientation = orientation

class PickUpMotionDescription(MotionDesignatorDescription):
	object: Object
	arm: str
	gripper: str
	grasp: str
	def __init__(self, object, grasp=None, arm=None, resolver="grounding"):
		super().__init__(resolver)
		self.cmd = 'pick-up'
		self.object = object
		self.arm = arm
		self.gripper = None
		self.grasp = grasp

class PlaceMotionDescription(MotionDesignatorDescription):
	object: Object
	target: list
	arm: str
	gripper: str
	def __init__(self, object, target, arm=None, resolver="grounding"):
		super().__init__(resolver)
		self.cmd = 'place'
		self.object = object
		self.target = target
		self.arm = arm
		self.gripper = None

class AccessingMotionDescription(MotionDesignatorDescription):
	drawer_joint: str
	drawer_handle: str
	part_of: Object
	arm: str
	distance: float
	gripper: str
	def __init__(self, drawer_joint, drawer_handle, part_of, arm=None, distance=0.3, resolver="grounding"):
		super().__init__(resolver)
		self.cmd = 'access'
		self.drawer_joint = drawer_joint
		self.drawer_handle = drawer_handle
		self.part_of = part_of
		self.arm = arm
		self.distance = distance
		self.gripper = None

class MoveTCPMotionDescription(MotionDesignatorDescription):
	target: list
	arm: str
	def __init__(self, target, arm=None, resolver="grounding"):
		super().__init__(resolver)
		self.cmd = 'move-tcp'
		self.target = target
		self.arm = arm

class LookingMotionDescription(MotionDesignatorDescription):
	target: list
	object: Object
	def __init__(self, target=None, object=None, resolver="grounding"):
		super().__init__(resolver)
		self.cmd = 'looking'
		self.target = target
		self.object = object

class MoveGripperMotionDescription(MotionDesignatorDescription):
	motion: str
	gripper: str
	def __init__(self, motion, gripper, resolver="grounding"):
		super().__init__(resolver)
		self.cmd = 'move-gripper'
		self.motion = motion
		self.gripper = gripper

class DetectingMotionDescription(MotionDesignatorDescription):
	object_type: str
	cam_frame: str
	front_facing_axis: list
	def __init__(self, object_type, cam_frame=None, front_facing_axis=None, resolver="grounding"):
		super().__init__(resolver)
		self.cmd = 'detecting'
		self.object_type = object_type
		self.cam_frame = cam_frame
		self.front_facing_axis = front_facing_axis

class MoveArmJointsMotionDescription(MotionDesignatorDescription):
	left_arm_config: str
	right_arm_config: str
	left_arm_poses: dict
	right_arm_poses: dict
	def __init__(self, left_arm_config=None, right_arm_config=None, left_arm_poses=None, right_arm_poses=None,
				 resolver="grounding"):
		super().__init__(resolver)
		self.cmd = 'move-joints'
		self.left_arm_config = left_arm_config
		self.right_arm_config = right_arm_config
		self.left_arm_poses = left_arm_poses
		self.right_arm_poses = right_arm_poses

class WorldStateDetectingMotionDescription(MotionDesignatorDescription):
	object_type: str
	def __init__(self, object_type, resolver="grounding"):
		super().__init__(resolver)
		self.cmd = 'world-state-detecting'
		self.object_type = object_type
