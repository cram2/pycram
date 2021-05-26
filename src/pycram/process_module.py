"""Implementation of process modules.

Classes:
ProcessModule -- implementation of process modules.
"""
from .fluent import Fluent

class ProcessModule:
	"""Implementation of process modules.

	Process modules are the part that communicate with the outer world to execute designators.

	Variables:
	resolvers -- list of all process module resolvers.

	Functions:
	perform -- automatically choose a process module and execute the given designator.

	Methods:
	execute -- execute the given designator.
	"""

	resolvers = []
	"""List of all process module resolvers. Process module resolvers are functions which take a designator as argument and return a process module."""
	robot_type = ""
	"""The type of the robot, either real or simulated. Is used to determine which Process Module is choosen for execution."""


	@staticmethod
	def perform(designator):
		"""Automatically choose a process module and execute the given designator.

		Arguments:
		designator -- the designator to choose the process module for and to execute.
		"""
		for resolver in ProcessModule.resolvers:
			pm = resolver(designator)

			if pm is not None:
				return pm.execute(designator)

	def __init__(self):
		"""Create a new process module."""
		self._running = Fluent(False)
		self._designators = []

	def _execute(self, designator):
		"""This is a helper method for internal usage only.

		This method is to be overwritten instead of the execute method.
		"""
		pass

	def execute(self, designator):
		"""Execute the given designator. If the process module is already executing another designator, it queues the given designator and executes them in order.

		Arguments:
		designator -- the designator to execute.
		"""
		self._designators.append(designator)
		(self._running == False).wait_for()
		self._running.set_value(True)
		designator = self._designators[0]
		ret = self._execute(designator)
		self._designators.remove(designator)
		self._running.set_value(False)
		return ret

class real_robot():
	def __init__(self):
		self.pre = ""
	def __enter__(self):
		self.pre = ProcessModule.robot_type
		ProcessModule.robot_type = "real"
	def __exit__(self, type, value, traceback):
		ProcessModule.robot_type = self.pre

class simulated_robot():
	def __init__(self):
		self.pre = ""
	def __enter__(self):
		self.pre = ProcessModule.robot_type
		ProcessModule.robot_type = "simulated"
	def __exit__(self, type, value, traceback):
		ProcessModule.robot_type = self.pre

def with_real_robot(func):
	def wrapper(*args, **kwargs):
		pre = ProcessModule.robot_type
		ProcessModule.robot_type = "real"
		func(*args, **kwargs)
		ProcessModule.robot_type = pre
	return wrapper

def with_simulated_robot(func):
	def wrapper(*args, **kwargs):
		pre = ProcessModule.robot_type
		ProcessModule.robot_type = "simulated"
		func(*args, **kwargs)
		ProcessModule.robot_type = pre
	return wrapper
