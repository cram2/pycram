:py:mod:`pycram.datastructures.enums`
=====================================

.. py:module:: pycram.datastructures.enums

.. autoapi-nested-parse::

   Module holding all enums of PyCRAM.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.datastructures.enums.Arms
   pycram.datastructures.enums.TaskStatus
   pycram.datastructures.enums.JointType
   pycram.datastructures.enums.Grasp
   pycram.datastructures.enums.ObjectType
   pycram.datastructures.enums.State
   pycram.datastructures.enums.Shape
   pycram.datastructures.enums.WorldMode
   pycram.datastructures.enums.AxisIdentifier
   pycram.datastructures.enums.GripperState




.. py:class:: Arms


   Bases: :py:obj:`enum.Enum`

   Enum for Arms.

   .. py:attribute:: LEFT

      

   .. py:attribute:: RIGHT

      

   .. py:attribute:: BOTH

      


.. py:class:: TaskStatus


   Bases: :py:obj:`enum.Enum`

   Enum for readable descriptions of a tasks' status.

   .. py:attribute:: CREATED
      :value: 0

      

   .. py:attribute:: RUNNING
      :value: 1

      

   .. py:attribute:: SUCCEEDED
      :value: 2

      

   .. py:attribute:: FAILED
      :value: 3

      


.. py:class:: JointType


   Bases: :py:obj:`enum.Enum`

   Enum for readable joint types.

   .. py:attribute:: REVOLUTE
      :value: 0

      

   .. py:attribute:: PRISMATIC
      :value: 1

      

   .. py:attribute:: SPHERICAL
      :value: 2

      

   .. py:attribute:: PLANAR
      :value: 3

      

   .. py:attribute:: FIXED
      :value: 4

      

   .. py:attribute:: UNKNOWN
      :value: 5

      

   .. py:attribute:: CONTINUOUS
      :value: 6

      

   .. py:attribute:: FLOATING
      :value: 7

      


.. py:class:: Grasp


   Bases: :py:obj:`enum.Enum`

   Enum for Grasp orientations.

   .. py:attribute:: FRONT
      :value: 0

      

   .. py:attribute:: LEFT
      :value: 1

      

   .. py:attribute:: RIGHT
      :value: 2

      

   .. py:attribute:: TOP
      :value: 3

      


.. py:class:: ObjectType


   Bases: :py:obj:`enum.Enum`

   Enum for Object types to easier identify different objects

   .. py:attribute:: METALMUG

      

   .. py:attribute:: PRINGLES

      

   .. py:attribute:: MILK

      

   .. py:attribute:: SPOON

      

   .. py:attribute:: BOWL

      

   .. py:attribute:: BREAKFAST_CEREAL

      

   .. py:attribute:: JEROEN_CUP

      

   .. py:attribute:: ROBOT

      

   .. py:attribute:: ENVIRONMENT

      

   .. py:attribute:: GENERIC_OBJECT

      

   .. py:attribute:: HUMAN

      


.. py:class:: State


   Bases: :py:obj:`enum.Enum`

   Enumeration which describes the result of a language expression.

   .. py:attribute:: SUCCEEDED
      :value: 1

      

   .. py:attribute:: FAILED
      :value: 0

      

   .. py:attribute:: RUNNING
      :value: 2

      

   .. py:attribute:: INTERRUPTED
      :value: 3

      


.. py:class:: Shape


   Bases: :py:obj:`enum.Enum`

   Enum for visual shapes of objects

   .. py:attribute:: SPHERE
      :value: 2

      

   .. py:attribute:: BOX
      :value: 3

      

   .. py:attribute:: CYLINDER
      :value: 4

      

   .. py:attribute:: MESH
      :value: 5

      

   .. py:attribute:: PLANE
      :value: 6

      

   .. py:attribute:: CAPSULE
      :value: 7

      


.. py:class:: WorldMode


   Bases: :py:obj:`enum.Enum`

   Enum for the different modes of the world.

   .. py:attribute:: GUI
      :value: 'GUI'

      

   .. py:attribute:: DIRECT
      :value: 'DIRECT'

      


.. py:class:: AxisIdentifier


   Bases: :py:obj:`enum.Enum`

   Enum for translating the axis name to a vector along that axis.

   .. py:attribute:: X
      :value: (1, 0, 0)

      

   .. py:attribute:: Y
      :value: (0, 1, 0)

      

   .. py:attribute:: Z
      :value: (0, 0, 1)

      


.. py:class:: GripperState


   Bases: :py:obj:`enum.Enum`

   Enum for the different motions of the gripper.

   .. py:attribute:: OPEN

      

   .. py:attribute:: CLOSE

      


