:py:mod:`pycram.external_interfaces.move_base`
==============================================

.. py:module:: pycram.external_interfaces.move_base


Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   pycram.external_interfaces.move_base.create_nav_action_client
   pycram.external_interfaces.move_base.init_nav_interface
   pycram.external_interfaces.move_base.query_pose_nav
   pycram.external_interfaces.move_base.cancel_nav



Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.external_interfaces.move_base.nav_action_client
   pycram.external_interfaces.move_base.is_init


.. py:data:: nav_action_client

   

.. py:data:: is_init
   :value: False

   

.. py:function:: create_nav_action_client() -> actionlib.SimpleActionClient

   Creates a new action client for the move_base interface.


.. py:function:: init_nav_interface(func: Callable) -> Callable

   Ensures initialization of the navigation interface before function execution.


.. py:function:: query_pose_nav(navpose: geometry_msgs.msg.PoseStamped)

   Sends a goal to the move_base service, initiating robot navigation to a given pose.


.. py:function:: cancel_nav()

   Cancels the current navigation goal.


