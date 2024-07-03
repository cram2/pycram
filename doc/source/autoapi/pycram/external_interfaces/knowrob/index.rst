:py:mod:`pycram.external_interfaces.knowrob`
============================================

.. py:module:: pycram.external_interfaces.knowrob


Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   pycram.external_interfaces.knowrob.all_solutions
   pycram.external_interfaces.knowrob.once
   pycram.external_interfaces.knowrob.load_beliefstate
   pycram.external_interfaces.knowrob.clear_beliefstate
   pycram.external_interfaces.knowrob.load_owl
   pycram.external_interfaces.knowrob.new_iri
   pycram.external_interfaces.knowrob.object_type
   pycram.external_interfaces.knowrob.instances_of
   pycram.external_interfaces.knowrob.object_pose
   pycram.external_interfaces.knowrob.grasp_pose
   pycram.external_interfaces.knowrob.knowrob_string_to_pose



Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.external_interfaces.knowrob.SCRIPT_DIR
   pycram.external_interfaces.knowrob.neem_interface
   pycram.external_interfaces.knowrob.logger


.. py:data:: SCRIPT_DIR

   

.. py:data:: neem_interface

   

.. py:data:: logger

   

.. py:function:: all_solutions(q)


.. py:function:: once(q) -> typing_extensions.Union[typing_extensions.List, typing_extensions.Dict]


.. py:function:: load_beliefstate(path: str)


.. py:function:: clear_beliefstate()


.. py:function:: load_owl(path, ns_alias=None, ns_url=None)

   Example: load_owl("package://external_interfaces/owl/maps/iai_room_v1.owl", "map", "http://knowrob.org/kb/v1/IAI-Kitchen.owl#")
   :param str path: path to log folder
   :rtype: bool


.. py:function:: new_iri(owl_class: str)


.. py:function:: object_type(object_iri: str) -> str

   :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base


.. py:function:: instances_of(type_: str) -> typing_extensions.List[str]

   :param type_: An object type (i.e. class)


.. py:function:: object_pose(object_iri: str, reference_cs: str = 'world', timestamp=None) -> typing_extensions.List[float]

   :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
   :param reference_cs: The coordinate system relative to which the pose should be defined


.. py:function:: grasp_pose(object_iri: str) -> typing_extensions.List[float]


.. py:function:: knowrob_string_to_pose(pose_as_string: str) -> typing_extensions.List[float]


