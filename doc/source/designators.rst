.. _designators:

===========
Designators
===========

Designators are CRAMs and PyCRAMs way of representing actions, motions, objects and locations.

In general, CRAM-Designators consist of a description and a specified element.
Descriptions describe sets of designators and designators are one thing in the described set.

.. image:: ../images/designators.png
   :alt: Schematic representation of Designators.

There are four types of designators in PyCRAM:

 - :mod:`pycram.designators.action_designator`
 - :mod:`pycram.designators.object_designator`
 - :mod:`pycram.designators.location_designator`
 - :mod:`pycram.designators.motion_designator`

Object Designator
=================

Object designators correspond to objects in (simulated) world.
The description of object designators can take names and types that the object should match.
The :meth:`pycram.designators.object_designator.ObjectDesignatorDescription.ground` method returns an object with all
its data attached that matches the description.
The :meth:`pycram.designators.object_designator.ObjectDesignatorDescription.__iter__` method iterates over all objects
that match the description.

Contributing Object Designators
-------------------------------
Object Designators should always be part of an object designator description.
The general class structure is seen in :mod:`pycram.designators.object_designator.ObjectDesignatorDescription`.
New object description need to inherit from the general object description. If the object they ground to differs from
the base object, a `dataclass <https://docs.python.org/3/library/dataclasses.html>`_. should be created inside the new
description. The dataclass is one element that matches the description.
If ORM logging of the new objects is wanted a ``to_sql()`` and ``insert()`` method has to be implemented
(see :ref:`orm` for more details).


Action Designator
=================
Action designators describe complex actions that are executable for an agent. Action designators can be seen as higher
level plans that include failure handling and parametrization.
An action

Motion Designator
=================
Motion designators describe atomic actions that are executable for an agent

Location Designator
===================

