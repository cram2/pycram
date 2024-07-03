:py:mod:`pycram.ros.robot_state_updater`
========================================

.. py:module:: pycram.ros.robot_state_updater


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.ros.robot_state_updater.RobotStateUpdater




.. py:class:: RobotStateUpdater(tf_topic: str, joint_state_topic: str)


   Updates the robot in the World with information of the real robot published to ROS topics.
   Infos used to update the robot are:

       * The current pose of the robot
       * The current joint state of the robot

   The robot state updater uses a TF topic and a joint state topic to get the current state of the robot.

   :param tf_topic: Name of the TF topic, needs to publish geometry_msgs/TransformStamped
   :param joint_state_topic: Name of the joint state topic, needs to publish sensor_msgs/JointState

   .. py:method:: _subscribe_tf(msg: geometry_msgs.msg.TransformStamped) -> None

      Callback for the TF timer, will do a lookup of the transform between map frame and the robot base frame.

      :param msg: TransformStamped message published to the topic


   .. py:method:: _subscribe_joint_state(msg: sensor_msgs.msg.JointState) -> None

      Sets the current joint configuration of the robot in the world to the configuration published on the
      topic. Since this uses rospy.wait_for_message which can have errors when used with threads there might be an
      attribute error in the rospy implementation.

      :param msg: JointState message published to the topic.


   .. py:method:: _stop_subscription() -> None

      Stops the Timer for TF and joint states and therefore the updating of the robot in the world.



