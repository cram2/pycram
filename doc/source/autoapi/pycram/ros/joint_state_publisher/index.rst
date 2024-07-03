:py:mod:`pycram.ros.joint_state_publisher`
==========================================

.. py:module:: pycram.ros.joint_state_publisher


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.ros.joint_state_publisher.JointStatePublisher




.. py:class:: JointStatePublisher(joint_state_topic='/pycram/joint_state', interval=0.1)


   Joint state publisher for the robot currently loaded in the World

   Robot object is from :py:attr:`~pycram.world.World.robot` and current joint states are published to
   the given joint_state_topic as a JointState message.

   :param joint_state_topic: Topic name to which the joint states should be published
   :param interval: Interval at which the joint states should be published, in seconds

   .. py:method:: _publish() -> None

      Publishes the current joint states of the :py:attr:`~pycram.world.World.robot` in an infinite loop.
      The joint states are published as long as the kill_event is not set by :py:meth:`~JointStatePublisher._stop_publishing`


   .. py:method:: _stop_publishing() -> None

      Sets the kill_event to terminate the publishing thread and joins the thread.



