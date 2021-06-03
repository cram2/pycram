from .designator import Designator, DesignatorError, ResolutionError
from .helper import GeneratorList
from .bullet_world import Object
from inspect import isgenerator, isgeneratorfunction
from typing import get_type_hints
from copy import copy


class MotionDesignator(Designator):
	"""
	Implementation of motion designators.

	Variables:
	resolvers -- list of all motion designator resolvers.
	"""

	resolvers = []
	"""List of all motion designator resolvers. Motion designator resolvers are functions which take a designator as argument and return a list of solutions. A solution can also be a generator."""

	def __init__(self, description, parent = None):
		self._solutions = None
		self._index = 0
		Designator.__init__(self, description, parent)

	def _reference(self):
		if self._solutions is None:
			def generator():
				for resolver in MotionDesignator.resolvers:
					for solution in resolver(self):
						if isgeneratorfunction(solution):
							solution = solution()

						if isgenerator(solution):
							while True:
								try:
									yield next(solution)
								except StopIteration:
									break
						else:
							yield solution

			self._solutions = GeneratorList(generator)

		if self._data is not None:
			return self._data

		try:
			self._data = self._solutions.get(self._index)
			return self._data
		except StopIteration:
			raise DesignatorError('Cannot resolve motion designator')

	def next_solution(self):
		try:
			self.reference()
		except DesignatorError:
			pass

		if self._solutions.has(self._index + 1):
			desig = MotionDesignator(self._properties, self)
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


class MotionDesignatorDescription:
	cmd: str
	def make_dictionary(self, properties):
		"""
		Creates a dictionary of this description with only the given properties
		included.
		:param properties: A list of properties that should be included in the dictionary.
							The given properties have to be an attribute of this description.
		:return: A dictionary with the properties as keys.
		"""
		attributes = self.__dict__
		ret = {}
		for att in attributes.keys():
			if att in properties:
				ret[att] = attributes[att]
		return ret

	def _check_properties(self, desig, exclude=[]):
		"""
		Checks the properties of this description. It will be checked if any attribute is
		None and if any attribute has to wrong type according to the type hints in
		the description class.
		It is possible to provide a list of attributes which should not be checked.
		:param desig: The current type of designator, will be used when raising an
						Exception as output.
		:param exclude: A list of properties which should not be checked.
		"""
		right_types = get_type_hints(type(self))
		attributes = self.__dict__
		missing = []
		wrong_type = {}
		current_type = {}
		for k in attributes.keys():
			if attributes[k] == None and not attributes[k] in exclude:
				missing.append(k)
			elif type(attributes[k]) != right_types[k] and not attributes[k] in exclude:
				wrong_type[k] = right_types[k]
				current_type[k] = type(attributes[k])
		if missing != [] or wrong_type != {}:
			raise ResolutionError(missing, wrong_type, current_type, desig)

	def ground(self):
		"""
		Should be overriten with an actual grounding function which inferres
		missing properties.
		"""
		return self

	def get_slots(self):
		"""
		Returns a list of all slots of this description. Can be used for inspecting
		different descriptions and debugging.
		:return: A list of all slots.
		"""
		return list(self.__dict__.keys()).remove('cmd')

	def copy(self):
		return copy(self)

class MoveMotionDescription(MotionDesignatorDescription):
	"""
	Definition of types. Is used in _check_missing_properties for evaluating
	the types of given properties.
	"""
	target: list
	orientation: list
	def __init__(self, target, orientation=None):
		self.cmd = "navigate"
		self.target = target
		self.orientation = orientation

class PickUpMotionDescription(MotionDesignatorDescription):
	object: Object
	arm: str
	gripper: str
	def __init__(self, object, arm=None):
		self.cmd = 'pick'
		self.object = object
		self.arm = arm
		self.gripper = None

class PlaceMotionDescription(MotionDesignatorDescription):
	object: Object
	target: list
	arm: str
	gripper: str
	def __init__(self, object, target, arm=None):
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
	def __init__(self, drawer_joint, drawer_handle, part_of, arm=None, distance=0.3):
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
	def __init__(self, target, arm=None):
		self.cmd = 'move-tcp'
		self.target = target
		self.arm = arm

class LookingMotionDescription(MotionDesignatorDescription):
	target: list
	object: Object
	def __init__(self, target=None, object=None):
		self.cmd = 'looking'
		self.target = target
		self.object = object

class MoveGripperMotionDescription(MotionDesignatorDescription):
	motion: str
	gripper: str
	def __init__(self, motion, gripper):
		self.cmd = 'move-gripper'
		self.motion = motion
		self.gripper = gripper

class DetectingMotionDescription(MotionDesignatorDescription):
	object_type: str
	cam_frame: str
	front_facing_axis: list
	def __init__(self, object_type, cam_frame=None, front_facing_axis=None):
		self.cmd = 'detecting'
		self.object_type = object_type
		self.cam_frame = cam_frame
		self.front_facing_axis = front_facing_axis

class MoveArmJointsMotionDescription(MotionDesignatorDescription):
	left_arm_config: str
	right_arm_config: str
	left_arm_poses: dict
	right_arm_poses: dict
	def __init__(self, left_arm_config=None, right_arm_config=None, left_arm_poses=None, right_arm_poses=None):
		self.cmd = 'move-joints'
		self.left_arm_config = left_arm_config
		self.right_arm_config = right_arm_config
		self.left_arm_poses = left_arm_poses
		self.right_arm_poses = right_arm_poses

class WorldStateDetectingMotionDescription(MotionDesignatorDescription):
	object_type: str
	def __init__(self, object_type):
		self.cmd = 'world-state-detecting'
		self.object_type = object_type
