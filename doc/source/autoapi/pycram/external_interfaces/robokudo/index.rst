:py:mod:`pycram.external_interfaces.robokudo`
=============================================

.. py:module:: pycram.external_interfaces.robokudo


Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   pycram.external_interfaces.robokudo.init_robokudo_interface
   pycram.external_interfaces.robokudo.create_robokudo_action_client
   pycram.external_interfaces.robokudo.msg_from_obj_desig
   pycram.external_interfaces.robokudo.make_query_goal_msg
   pycram.external_interfaces.robokudo.query



Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.external_interfaces.robokudo.robokudo_action_client


.. py:data:: robokudo_action_client

   

.. py:function:: init_robokudo_interface(func: typing_extensions.Callable) -> typing_extensions.Callable

   Tries to import the RoboKudo messages and with that initialize the RoboKudo interface.


.. py:function:: create_robokudo_action_client() -> typing_extensions.Callable

   Creates a new action client for the RoboKudo query interface and returns a function encapsulating the action client.
   The returned function can be called with an ObjectDesigantor as parameter and returns the result of the action client.

   :return: A callable function encapsulating the action client


.. py:function:: msg_from_obj_desig(obj_desc: pycram.designator.ObjectDesignatorDescription) -> robokudo_msgs.msg.ObjectDesignator

   Creates a RoboKudo Object designator from a PyCRAM Object Designator description

   :param obj_desc: The PyCRAM Object designator that should be converted
   :return: The RobotKudo Object Designator for the given PyCRAM designator


.. py:function:: make_query_goal_msg(obj_desc: pycram.designator.ObjectDesignatorDescription) -> robokudo_msgs.msg.QueryGoal

   Creates a QueryGoal message from a PyCRAM Object designator description for the use of Querying RobotKudo.

   :param obj_desc: The PyCRAM object designator description that should be converted
   :return: The RoboKudo QueryGoal for the given object designator description


.. py:function:: query(object_desc: pycram.designator.ObjectDesignatorDescription) -> pycram.designator.ObjectDesignatorDescription.Object

   Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
   For sending the query to RoboKudo a simple action client will be created and the Object designator description is
   sent as a goal.

   :param object_desc: The object designator description which describes the object that should be perceived
   :return: An object designator for the found object, if there was an object that fitted the description.


