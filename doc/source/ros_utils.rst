=========
ROS Utils
=========

PyCRAM provides a number of utils to interact with the ROS network. The utils are:

* A TF Broadcaster
* A Joint State Publisher
* A Simulated Force Torque Sensor
* A Visualisation Marker Publisher
* A Robot State Updater

These site will go over all utils what they do and how to use them. All ROS utils presented here
will publish continuously in a new thread. You can either stop the publishing by calling the
```stop_publishing``` or the thread will terminate automatically once the process is ended.

--------------
TF Broadcaster
--------------

The TF broadcaster broadcasts the transformations to every object and every link in the
BulletWorld to the TF topic. This allows other ROS nodes to listen to the TF topic and
transform their poses to the TF frames published in the transforms on the TF topic.

The broadcaster publishes transforms in a specific interval, this interval can be specified
when creating the broadcaster.

.. code-block:: python

    from pycram.ros.tf_broadcaster import TFBroadcaster

    broadcaster = TFBroadcaster()

The broadcaster allows to specify a projection namespace, the projection namespace will be
prefixed before the TF frames. Furthermore, you can specify an odom frame and the interval
at which the transforms will be published in seconds.

---------------------
Joint State Publisher
---------------------

The joint state publisher publishes the current joint positions for every joint of the
currently loaded robot. Furthermore, allows the joint state publisher to specify a topic
to publish as well as an interval at which the position should be published.

.. code-block:: python

    from pycram.ros.joint_state_publisher import JointStatePublisher

    joint_publisher = JointStatePublisher("joint_states", 0.1)



-------------------
Force Torque Sensor
-------------------

PyBullet, the underlying simulation framework, allows to simulate a force torque sensor for
a given joint. The results from this simulated force torque sensor are then published to
a ROS topic of type geometry_msgs/WrenchStamped. Furthermore, allows the force torque sensor
class to specify a topic as well as an interval in seconds.

.. code-block:: python

     from pycram.ros.force_torque_sensor import ForceTorqueSensor

     ft_sensor = ForceTorqueSensor("l_wrist_roll_joint", "/pycram/fts", 0.1)


------------------------------
Visualisation Marker Publisher
------------------------------
The visualisation marker publisher sends an array of visualisation marker to a ROS topic which can then be
rendered by RVIZ for example. The array consists of one Marker per link for each Object in the Bullet World, with
each Object creating its own namespace. The visualisation marker publisher has to instantiated once and then
publishes constantly in the background.

.. code-block:: python

    from pycram.ros.viz_marker_publisher import VizMarkerPublisher

    v = VizMarkerPublisher()

-------------------
Robot State Updater
-------------------
The robot state updater is intended for working with a real robot, since the robot in the bullet world should mimic
the real robot there has to be a module which takes care of updating the state of the robot in the BulletWorld. The
robot state updated hooks into the TF and joint_state topic to get the current position as well as the current joint
configuration of the robot and applies it to the robt in the BulletWorld.

As all the other ROS utils the robot state updater only needs to be initialized and then takes care of himself.

To initialize the robot state updater a TF topic as well as a joint state topic have to be provided.

.. code-block:: python

    from pycram.ros.robot_state_updater import RobotStateUpdater

    r = RobotStateUpdater("/tf", "/joint_states")
