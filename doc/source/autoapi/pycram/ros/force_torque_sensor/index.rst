:py:mod:`pycram.ros.force_torque_sensor`
========================================

.. py:module:: pycram.ros.force_torque_sensor


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.ros.force_torque_sensor.ForceTorqueSensor




.. py:class:: ForceTorqueSensor(joint_name, fts_topic='/pycram/fts', interval=0.1)


   Simulated force-torque sensor for a joint with a given name.
   Reads simulated forces and torques at that joint from world and publishes geometry_msgs/Wrench messages
   to the given topic.

   The given joint_name has to be part of :py:attr:`~pycram.world.World.robot` otherwise a
   RuntimeError will be raised.

   :param joint_name: Name of the joint for which force-torque should be simulated
   :param fts_topic: Name of the ROS topic to which should be published
   :param interval: Interval at which the messages should be published, in seconds

   .. py:method:: _publish() -> None

      Continuously publishes the force-torque values for the simulated joint. Values are published as long as the
      kill_event is not set.


   .. py:method:: _stop_publishing() -> None

      Sets the kill_event and therefore terminates the Thread publishing the force-torque values as well as join the
      threads.



