"""Implementation of the CRAM language.

Macros:
seq -- macro to execute statements sequentially and fail if one fails, succeed after all succeeded.
par -- macro to execute statements in parallel and fail if one fails, succeed after all succeeded.
pursue -- macro to execute statements in parallel and fail if one fails, succeed after one succeeded.
try_all -- macro to execute statements in parallel and fail if all fail, succeed after one succeeded.
try_in_order -- macro to execute statements sequentially and fail if all fail, succeed after one succeeded.
failure_handling -- macro to wrap the body into a function named retry which can be called to execute it again.

Classes:
State -- enumeration to describe the result of a macro.
"""
import ast
from .fluent import Fluent
from .helper import _block
from enum import Enum
from macropy.core.macros import Macros
from macropy.core.hquotes import macros, hq
from macropy.core.quotes import macros, ast_literal, q
from threading import Thread

macros = Macros()
"""Must be imported before macros defined in this module can be imported."""

def _state(target, state = None):
	"""This is a helper function for internal usage only.

	Set the state which macros return as result. If it's already set, nothing happens. If the given argument "state" is None, the current state is returned.

	Arguments:
	target -- the varibale which stores the state as fluent.
	state -- the state (default is None).
	"""
	target_load = ast.Name(target.id, ast.Load())

	if state is None:
		return q[ast_literal[target_load].get_value()]

	with hq as tree:
		if not ast_literal[target_load].get_value():
			ast_literal[target_load].set_value(state)

	return tree

def _init(target, threads = True):
	"""This is a helper function for internal usage only.

	Initialize variables which macros need.

	Arguments:
	target -- the variable which stores the state.
	threads -- boolean value describing whether the macro needs a list to store threads in.
	"""
	with hq as tree:
		ast_literal[target] = Fluent()
		_exceptions = []

	if threads:
		with q as temp_tree:
			_threads = []

		tree.append(temp_tree)

	return tree

def _exceptions(tree, args):
	"""This is a helper function for internal usage only.

	Store the list of exceptions if the argument is given.

	Arguments:
	tree -- the syntax tree to append the subtree to.
	args -- arguments passed to the macro.
	"""
	if len(args) > 0:
		with hq as new_tree:
			ast_literal[ast.Name(args[0].id, ast.Store())] = unhygienic[_exceptions]

		tree.append(new_tree)

def _thread(tree):
	"""This is a helper function for internal usage only.

	Create a thread and start it.

	Arguments:
	tree -- the syntax tree to append the subtree to.
	"""
	with hq as new_tree:
		_thread = Thread(target = unhygienic[_func])
		unhygienic[_threads].append(_thread)
		_thread.start()

	tree.append(new_tree)

def _join(tree):
	"""This is a helper function for internal usage only.

	Join all threads (wait for them to finish).

	Arguments:
	tree -- the syntax tree to append the subtree to.
	"""
	with hq as new_tree:
		for _thread in unhygienic[_threads]:
			_thread.join()

	tree.append(new_tree)

@macros.block
def seq(tree, target, args, **kw):
	"""Execute statements sequentially and fail if one fails, succeed after all succeeded. If one failed, the others are not executed anymore.
	The result is returned as fluent. One can access the state within the macro, too.
	Exceptions do not terminate the current thread, they get stored as list into a variable passed as optional argument instead.

	This macro can also be used to wrap multiple statements into a single block and thus treating multiple statements as one.

	Example usage:
	with seq(exceptions) as state:
		statement1
		statement2
		...

	Arguments:
	exceptions -- variable to store a list of exceptions in (optional).
	"""
	new_tree = _init(target, False)

	for statement in tree:
		with hq as temp_tree:
			if ast_literal[_state(target)] is None:
				try:
					ast_literal[statement]
				except Exception as e:
					ast_literal[_state(target, State.FAILED)]
					unhygienic[_exceptions].append(e)

		new_tree.append(temp_tree);

	new_tree.append(_state(target, State.SUCCEEDED))
	_exceptions(new_tree, args)
	return _block(new_tree)

@macros.block
def par(tree, target, args, **kw):
	"""Execute statements in parallel and fail if one fails, succeed after all succeeded.
	The result is returned as fluent. One can access the state within the macro, too. This is especially useful to evaporate all statements if one finished.
	Exceptions do not terminate the current thread, they get stored as list into a variable passed as optional argument instead.

	Example usage:
	with par(exceptions) as state:
		statement1
		statement2
		...

	Arguments:
	exceptions -- variable to store a list of exceptions in (optional).
	"""
	new_tree = _init(target)

	for statement in tree:
		with hq as temp_tree:
			def _func():
				try:
					ast_literal[statement]
				except Exception as e:
					ast_literal[_state(target, State.FAILED)]
					unhygienic[_exceptions].append(e)

		new_tree.append(temp_tree)
		_thread(new_tree)

	_join(new_tree)
	new_tree.append(_state(target, State.SUCCEEDED))
	_exceptions(new_tree, args)
	return _block(new_tree)

@macros.block
def pursue(tree, target, args, **kw):
	"""Execute statements in parallel and fail if one fails, succeed after one succeeded.
	The result is returned as fluent. One can access the state within the macro, too. This is especially useful to evaporate all statements if one finished.
	Exceptions do not terminate the current thread, they get stored as list into a variable passed as optional argument instead.

	Example usage:
	with pursue(exceptions) as state:
		statement1
		statement2
		...

	Arguments:
	exceptions -- variable to store a list of exceptions in (optional).
	"""
	new_tree = _init(target)

	for statement in tree:
		with hq as temp_tree:
			def _func():
				try:
					ast_literal[statement]
					ast_literal[_state(target, State.SUCCEEDED)]
				except Exception as e:
					ast_literal[_state(target, State.FAILED)]
					unhygienic[_exceptions].append(e)

		new_tree.append(temp_tree)
		_thread(new_tree)

	_join(new_tree)
	_exceptions(new_tree, args)
	return _block(new_tree)

@macros.block
def try_all(tree, target, args, **kw):
	"""Execute statements in parallel and fail if all fail, succeed after one succeeded.
	The result is returned as fluent. One can access the state within the macro, too. This is especially useful to evaporate all statements if one finished.
	Exceptions do not terminate the current thread, they get stored as list into a variable passed as optional argument instead.

	Example usage:
	with try_all(exceptions) as state:
		statement1
		statement2
		...

	Arguments:
	exceptions -- variable to store a list of exceptions in (optional).
	"""
	new_tree = _init(target)

	for statement in tree:
		with hq as temp_tree:
			def _func():
				try:
					ast_literal[statement]
					ast_literal[_state(target, State.SUCCEEDED)]
				except Exception as e:
					unhygienic[_exceptions].append(e)

		new_tree.append(temp_tree)
		_thread(new_tree)

	_join(new_tree)
	new_tree.append(_state(target, State.FAILED))
	_exceptions(new_tree, args)
	return _block(new_tree)

@macros.block
def try_in_order(tree, target, args, **kw):
	"""Execute statements sequentially and fail if all fail, succeed after one succeeded. If one succeeded, the others are not executed anymore.
	The result is returned as fluent. One can access the state within the macro, too.
	Exceptions do not terminate the current thread, they get stored as list into a variable passed as optional argument instead.

	Example usage:
	with try_in_order(exceptions) as state:
		statement1
		statement2
		...

	Arguments:
	exceptions -- variable to store a list of exceptions in (optional).
	"""
	new_tree = _init(target, False)

	for statement in tree:
		with hq as temp_tree:
			if ast_literal[_state(target)] is None:
				try:
					ast_literal[statement]
					ast_literal[_state(target, State.SUCCEEDED)]
				except Exception as e:
					unhygienic[_exceptions].append(e)

		new_tree.append(temp_tree);

	new_tree.append(_state(target, State.FAILED))
	_exceptions(new_tree, args)
	return _block(new_tree)

@macros.block
def failure_handling(tree, args, **kw):
	"""Wrap the body into a function named retry which can be called to execute it again, for example in case of an exception being raised.
	The maximum number of retries can be specified.

	Example usage:
	with failure_handling(5):
		try:
			body
		except Exception as e:
			retry()

	Arguments:
	retries -- the maximum number of retries (not given or a negative value equals infinite).
	"""
	if len(args) == 0:
		args = [q[-1]]

	with hq as new_tree:
		_retries = Fluent(0)

		def retry():
			if ast_literal[args[0]] >= 0 and _retries.get_value() > ast_literal[args[0]]:
				return

			_retries.set_value(_retries.get_value() + 1)

			ast_literal[tree]

		retry()

	return _block(new_tree)

class State(Enum):
	"""Enumeration which describes the result of a macro.

	Fields:
	SUCCEEDED -- macro execution succeeded.
	FAILED -- macro execution failed.
	"""
	SUCCEEDED = 1
	FAILED = 2
