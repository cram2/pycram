===============
Troubleshooting
===============

This page contains the most common errors that could happen when using PyCRAM and how to resolve them.

------------------
Ros Init Exception
------------------
.. code-block:: Python

    ROSInitException: time is not initialized. Have you called init_node()

This exception usually occurs when trying to load an Object into the BulletWorld or when creating a Pose. The reason for
this exception is that the ROS node of PyCRAM could not be initialized, this is usually the case when the PyCRAM launch
file was not launched.

To solve this problem you just have to launch the ``ik_and_description`` file of PyCRAM. This can be done via the
following command.

.. code-block:: shell

    roslaunch pycram ik_and_description

----------------------------
Robot Description not Loaded
----------------------------

In PyCRAM a lot of things are base on the robot description that is currently loaded and in turn the robot description
that will be loaded depends on the robot description that is on the ROS parameter server at the time PyCRAM is started.

If you get an error that looks similar to the following exception the most likely case is that the robot description was
not loaded.

.. code-block:: python
   :emphasize-lines: 3

        763 with open(self.path) as f:
        764     self.urdf_object = URDF.from_xml_string(f.read())
    --> 765     if self.urdf_object.name == robot_description.name and not BulletWorld.robot:
        766         BulletWorld.robot = self
        768 self.links[self.urdf_object.get_root()] = -1

    AttributeError: 'NoneType' object has no attribute 'name'


There could be a few reasons for this behaviour:
   * The name of the URDF on the parameter server and the name in the PyCRAM robot description are different
      * The PyCRAM robot description is matched by the name of the robot in the URDF
      * You can check the assignment of the PyCRAM robot description in this function :func:`~pycram.robot_descriptions.update_robot_description` (there is a link to the source code)
   * You only started a roscore
   * You did not start the ``ik_and_description`` launch file


--------------
Stop Iteration
--------------

Stop iterations usually happen when you try to resolve a designator for which there is no solution or if you iterate over a
designator and reached the end of all possible solutions.

When you try to resolve a designator which has no solution the error will look something like this.

.. code-block:: python
   :emphasize-lines: 7

       753 def ground(self) -> Union[Object, bool]:
       754     """
       755     Return the first object from the bullet world that fits the description.
       756
       757     :return: A resolved object designator
       758     """
   --> 759     return next(iter(self))

   StopIteration:

If you encounter such an error the most likely reason is that you put the wrong arguments into your DesignatorDescription.
The best solution is to double check the input arguments of the DesignatorDescription.


-----------------------------------------
BulletWorld crashes after loading an URDF
-----------------------------------------

It can happen that the BulletWorld crashes when loading a URDF. This happens when loading a URDF with more than 127 links.
This is a limitation of PyBullet which can not deal with objects with more links.

Only real solution here is to either delete links such that you have at max 127 links or split up the URDF and load the
split parts individually.

----------------------------------------
Error when performing Actions or Motions
----------------------------------------

.. code-block:: python

        30 def perform(self):
        31     pm_manager = ProcessModuleManager.get_manager()
   ---> 32     return pm_manager.navigate().execute(self)

   AttributeError: 'NoneType' object has no attribute 'navigate'

If you get an error like this when trying to perform an action or motion designator, then you did not specify how the
designator should be executed. You can specify how the designator should be performed by using the simulated_robot or
real_robot environments. This is also explained in the `Action Designator Example <https://pycram.readthedocs.io/en/latest/notebooks/action_designator.html#Navigate-Action>`_.

.. code-block:: python

   with simulated_robot:
      NavigateAction([Pose()]).resolve().perform()


---------------------------
JntArray index out of Range
---------------------------
This error originates in the kdl_ik_service which is responsible for calculating ik solutions and usually looks like
this:
.. code-block:: pyhton

   ServiceException: service [/kdl_ik_service/get_ik] responded with an error: b'error processing request: JntArray index out of range'

This error can have two origins.

   * The URDF on the parameter server and the URDF of the robot for which an ik solution should be found are not identical
      * For example: The URDF on the parameter server is from the PR2 and a solution for the HSR should be found
   * The amount of names of joints and the amount of joint positions in the robot_state part of the request are not equal.