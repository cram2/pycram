===============
Troubleshooting
===============

This page contains the most common errors that could happen when using PyCRAM and how to resolve them.


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

