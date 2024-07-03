:py:mod:`pycram.utils`
======================

.. py:module:: pycram.utils

.. autoapi-nested-parse::

   Implementation of helper functions and classes for internal usage only.

   Functions:
   _block -- wrap multiple statements into a single block.

   Classes:
   GeneratorList -- implementation of generator list wrappers.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.utils.bcolors
   pycram.utils.GeneratorList
   pycram.utils.suppress_stdout_stderr



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.utils._apply_ik
   pycram.utils.axis_angle_to_quaternion



.. py:class:: bcolors


   Color codes which can be used to highlight Text in the Terminal. For example,
   for warnings.
   Usage:
   Firstly import the class into the file.
   print(f'{bcolors.WARNING} Some Text {bcolors.ENDC}')

   .. py:attribute:: HEADER
      :value: '\x1b[95m'

      

   .. py:attribute:: OKBLUE
      :value: '\x1b[94m'

      

   .. py:attribute:: OKCYAN
      :value: '\x1b[96m'

      

   .. py:attribute:: OKGREEN
      :value: '\x1b[92m'

      

   .. py:attribute:: WARNING
      :value: '\x1b[93m'

      

   .. py:attribute:: FAIL
      :value: '\x1b[91m'

      

   .. py:attribute:: ENDC
      :value: '\x1b[0m'

      

   .. py:attribute:: BOLD
      :value: '\x1b[1m'

      

   .. py:attribute:: UNDERLINE
      :value: '\x1b[4m'

      


.. py:function:: _apply_ik(robot: pycram.world_concepts.WorldObject, pose_and_joint_poses: typing_extensions.Tuple[pycram.datastructures.pose.Pose, typing_extensions.Dict[str, float]]) -> None

   Apllies a list of joint poses calculated by an inverse kinematics solver to a robot

   :param robot: The robot the joint poses should be applied on
   :param pose_and_joint_poses: The base pose and joint states as returned by the ik solver
   :return: None


.. py:class:: GeneratorList(generator: typing_extensions.Callable)


   Implementation of generator list wrappers.

   Generator lists store the elements of a generator, so these can be fetched multiple times.

   Methods:
   get -- get the element at a specific index.
   has -- check if an element at a specific index exists.

   Create a new generator list.

   Arguments:
   generator -- the generator to use.

   .. py:method:: get(index: int = 0)

      Get the element at a specific index or raise StopIteration if it doesn't exist.

      Arguments:
      index -- the index to get the element of.


   .. py:method:: has(index: int) -> bool

      Check if an element at a specific index exists and return True or False.

      Arguments:
      index -- the index to check for.



.. py:function:: axis_angle_to_quaternion(axis: typing_extensions.List, angle: float) -> typing_extensions.Tuple

   Convert axis-angle to quaternion.

   :param axis: (x, y, z) tuple representing rotation axis.
   :param angle: rotation angle in degree
   :return: The quaternion representing the axis angle


.. py:class:: suppress_stdout_stderr


   Bases: :py:obj:`object`

   A context manager for doing a "deep suppression" of stdout and stderr in
   Python, i.e. will suppress all prints, even if the print originates in a
   compiled C/Fortran sub-function.

   This will not suppress raised exceptions, since exceptions are printed
   to stderr just before a script exits, and after the context manager has
   exited (at least, I think that is why it lets exceptions through).
   Copied from https://stackoverflow.com/questions/11130156/suppress-stdout-stderr-print-from-python-functions

   .. py:method:: __enter__()


   .. py:method:: __exit__(*_)



