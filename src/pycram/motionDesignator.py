from .designator import Designator

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
        """
        To be overwritten by explicit ground function
        """
        pass

	def __str__(self):
		return "MotionDesignator({})".format(self._properties)


class MotionDesignatorDescription:
    function = None

    def ground(self):
        return self

	def make_dictionary(self, properties):
		"""Return the given properties as dictionary.

		Arguments:
		properties -- the properties to create a dictionary of. A property can
		be a tuple in which case its first value is the dictionary key and the
		second value is the dictionary value. Otherwise it's simply the dictionary
		key and the key of a property which is the dictionary value.
		"""
		attributes = self.__dict__

		for att in attributes.keys():
			if att not in properties:
				del attributes[att]

		return attributes

	def _check_missing_properties(self, exclude=[]):
		attributes = self.__dict__
		missing = []
		for k in attributes.keys():
			if attributes[k] == None and not in exclude:
				missing.append(k)
		return missing

class MoveMotionDescription(MotionDesignatorDescription):
    def __init__(self, target, orientation=None):
		self.cmd = 'navigate'
        self.target = target
        self.orientation = orientation

class PickUpMotionDescription(MotionDesignatorDescription):
    def __init__(self, object, arm=None):
		self.cmd = 'pick-up'
        self.object = object
        self.arm = arm

class PlaceMotionDescription(MotionDesignatorDescription):
    def __init__(self, object, target, arm=None):
		self.cmd = 'place'
        self.object = object
        self.target = target
        self.arm = arm

class AccessingMotinDescription(MotionDesignatorDescription):
    def __init__(self, drawer_joint, drawer_handle, part_of, arm=None, distance=0.3):
		self.cmd = 'access'
        self.drawer_joint = drawer_joint
        self.drawer_handle = drawer_handle
        self.part_of = part_of
        self.arm = arm
        self.distance = distance

class MoveTCPMotinDescription(MotionDesignatorDescription):
    def __init__(self, target, arm=None):
		self.cmd = 'move-tcp'
        self.target = target
        self.arm = arm

class LookingMotionDescription(MotionDesignatorDescription):
    def __init__(self, target=None, object=None):
		self.cmd = 'looking'
        self.target = target
        self.object = Object

class MoveGripperMotionDescription(MotionDesignatorDescription):
    def __init__(self, motion, gripper):
		self.cmd = 'move-gripper'
        self.motion = motion
        self.gripper = gripper

class DetectingMotionDescription(MotionDesignatorDescription):
    def __init__(self, object, cam_frame=None, front_facing_axis=None):
		self.cmd = 'detecting'
        self.object = object
        self.cam_frame = cam_frame
        self.front_facing_axis = front_facing_axis

class MoveArmJointsMotionDescription(MotionDesignatorDescription):
    def __init__(self, left_arm=None, right_arm=None):
		self.cmd = 'move-joints'
        self.left_arm = left_arm
        self.right_arm = right_arm

class WorldStateDetectingMotionDescription(MotionDesignatorDescription):
    def __init__(self, object):
		self.cmd = 'world-state-detecting'
        self.object = object
