"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from inspect import isgeneratorfunction
from macropy.core.quotes import macros, ast_literal, q


def transform(pose, transformation):
	res = [0, 0, 0]
	for i in range(0, 3):
		res[i] = pose[i] - transformation[i]
	return res


def _block(tree):
	"""Wrap multiple statements into a single block and return it.

	If macros themselves are not a single statement, they can't always be nested (for example inside the par macro which executes each statement in an own thread).

	Arguments:
	tree -- the tree containing the statements.
	"""
	with q as new_tree:
		# Wrapping the tree into an if block which itself is a statement that contains one or more statements.
		# The condition is just True and therefor makes sure that the wrapped statements get executed.
		if True:
			ast_literal[tree]

	return new_tree

class GeneratorList:
	"""Implementation of generator list wrappers.

	Generator lists store the elements of a generator, so these can be fetched multiple times.

	Methods:
	get -- get the element at a specific index.
	has -- check if an element at a specific index exists.
	"""

	def __init__(self, generator):
		"""Create a new generator list.

		Arguments:
		generator -- the generator to use.
		"""
		if isgeneratorfunction(generator):
			self._generator = generator()
		else:
			self._generator = generator

		self._generated = []

	def get(self, index = 0):
		"""Get the element at a specific index or raise StopIteration if it doesn't exist.

		Arguments:
		index -- the index to get the element of.
		"""
		while len(self._generated) <= index:
			self._generated.append(next(self._generator))

		return self._generated[index]

	def has(self, index):
		"""Check if an element at a specific index exists and return True or False.

		Arguments:
		index -- the index to check for.
		"""
		try:
			self.get(index)
			return True
		except StopIteration:
			return False
