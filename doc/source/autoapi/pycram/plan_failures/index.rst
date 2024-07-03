:py:mod:`pycram.plan_failures`
==============================

.. py:module:: pycram.plan_failures


Module Contents
---------------

.. py:exception:: PlanFailure(*args, **kwargs)


   Bases: :py:obj:`Exception`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: NotALanguageExpression(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: FailureDiagnosis(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: LowLevelFailure(*args, **kwargs)


   Bases: :py:obj:`FailureDiagnosis`

   Failure thrown by low-level modules: robot or projection PMs.

   Create a new plan failure.


.. py:exception:: ActionlibActionTimeout(*args, **kwargs)


   Bases: :py:obj:`LowLevelFailure`

   Failure thrown by low-level modules: robot or projection PMs.

   Create a new plan failure.


.. py:exception:: HighLevelFailure(*args, **kwargs)


   Bases: :py:obj:`FailureDiagnosis`

   Failure thrown by high-level modules, i.e. plans.

   Create a new plan failure.


.. py:exception:: DeliveringFailed(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when delivering plan completely gives up.

   Create a new plan failure.


.. py:exception:: ManipulationLowLevelFailure(*args, **kwargs)


   Bases: :py:obj:`LowLevelFailure`

   Thrown when a low-level, i.e. hardware related, failure is detected in a manipulation action.

   Create a new plan failure.


.. py:exception:: EnvironmentManipulationGoalNotReached(*args, **kwargs)


   Bases: :py:obj:`ManipulationLowLevelFailure`

   Thrown when door / drawer opening / closing goal is still not reached.

   Create a new plan failure.


.. py:exception:: EnvironmentManipulationImpossible(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when environment manipulation cannot be achieved.

   Create a new plan failure.


.. py:exception:: EnvironmentUnreachable(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when environment manipulation in collision or unreachable.

   Create a new plan failure.


.. py:exception:: FetchingFailed(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when fetching plan completely gives up.

   Create a new plan failure.


.. py:exception:: GripperLowLevelFailure(*args, **kwargs)


   Bases: :py:obj:`LowLevelFailure`

   Thrown when a failure involving the gripper hardware occurs.

   Create a new plan failure.


.. py:exception:: GripperClosedCompletely(*args, **kwargs)


   Bases: :py:obj:`GripperLowLevelFailure`

   Thrown when the gripper closed completely, despite not being expected to do so (e.g. because it should have
   grasped something).

   Create a new plan failure.


.. py:exception:: GripperGoalNotReached(*args, **kwargs)


   Bases: :py:obj:`GripperLowLevelFailure`

   Thrown when the gripper does not reach its goal.

   Create a new plan failure.


.. py:exception:: LookingHighLevelFailure(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   High-level failure produced when looking for an object, i.e. it is not a hardware issue but one relating to
   the looking task, its parameters, and how they relate to the environment.

   Create a new plan failure.


.. py:exception:: ManipulationGoalInCollision(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when executing a manipulation action results in a collision.

   Create a new plan failure.


.. py:exception:: ManipulationGoalNotReached(*args, **kwargs)


   Bases: :py:obj:`ManipulationLowLevelFailure`

   Thrown when after executing the action, goal is still not reached.

   Create a new plan failure.


.. py:exception:: IKError(pose, base_frame, tip_frame)


   Bases: :py:obj:`PlanFailure`

   Thrown when no inverse kinematics solution could be found

   Create a new plan failure.


.. py:exception:: ManipulationPoseUnreachable(*args, **kwargs)


   Bases: :py:obj:`ManipulationLowLevelFailure`

   Thrown when no IK solution can be found.

   Create a new plan failure.


.. py:exception:: NavigationHighLevelFailure(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   High-level failure produced while navigating the robot, i.e. it is not a hardware issue but one relating to
   the navigation task, its parameters, and how they relate to the environment.

   Create a new plan failure.


.. py:exception:: NavigationGoalInCollision(*args, **kwargs)


   Bases: :py:obj:`NavigationHighLevelFailure`

   Navigation goal cannot be reached because the goal itself is already occupied by some other object.

   Create a new plan failure.


.. py:exception:: NavigationLowLevelFailure(*args, **kwargs)


   Bases: :py:obj:`LowLevelFailure`

   Low-level failure produced while navigating the robot, i.e. some kind of hardware issue.

   Create a new plan failure.


.. py:exception:: NavigationGoalNotReached(*args, **kwargs)


   Bases: :py:obj:`NavigationLowLevelFailure`

   Thrown when the base moved as a result of the navigation action but the goal was not reached.

   Create a new plan failure.


.. py:exception:: NavigationPoseUnreachable(*args, **kwargs)


   Bases: :py:obj:`NavigationLowLevelFailure`

   Thrown when the goal pose for navigation is computed to be unreachable.

   Create a new plan failure.


.. py:exception:: ObjectNowhereToBeFound(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when the robot cannot find an object of a given description in its surroundings.

   Create a new plan failure.


.. py:exception:: ObjectUndeliverable(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when no base positioning can assure a reachable pose to place the object from.

   Create a new plan failure.


.. py:exception:: ObjectUnfetchable(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when no base positioning can assure a reachable pose to grasp the object from.

   Create a new plan failure.


.. py:exception:: ObjectUnreachable(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when no IK found for particular base pose.

   Create a new plan failure.


.. py:exception:: PerceptionLowLevelFailure(*args, **kwargs)


   Bases: :py:obj:`LowLevelFailure`

   Low-level failure produced while perceiving, i.e. some kind of hardware issue.

   Create a new plan failure.


.. py:exception:: PerceptionObjectNotFound(*args, **kwargs)


   Bases: :py:obj:`PerceptionLowLevelFailure`

   Thrown when an attempt to find an object by perception fails -- and this can still be interpreted as the robot
   not looking in the right direction, as opposed to the object being absent.

   Create a new plan failure.


.. py:exception:: PerceptionObjectNotInWorld(*args, **kwargs)


   Bases: :py:obj:`PerceptionLowLevelFailure`

   Thrown when an attempt to find an object by perception fails -- and this is because the object can be assumed
   absent or perhaps is known absent because of the setup of a simulation.

   Create a new plan failure.


.. py:exception:: SearchingFailed(*args, **kwargs)


   Bases: :py:obj:`HighLevelFailure`

   Thrown when searching plan completely gives up.

   Create a new plan failure.


.. py:exception:: TorsoLowLevelFailure(*args, **kwargs)


   Bases: :py:obj:`LowLevelFailure`

   Low-level failure produced while moving the torso, i.e. some kind of hardware issue.

   Create a new plan failure.


.. py:exception:: TorsoGoalNotReached(*args, **kwargs)


   Bases: :py:obj:`TorsoLowLevelFailure`

   Thrown when the torso moved as a result of a torso action but the goal was not reached.

   Create a new plan failure.


.. py:exception:: TorsoGoalUnreachable(*args, **kwargs)


   Bases: :py:obj:`TorsoLowLevelFailure`

   Thrown when the goal for the torso is computed to be unreachable.

   Create a new plan failure.


.. py:exception:: Task(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: Grasping(*args, **kwargs)


   Bases: :py:obj:`Task`

   
   Create a new plan failure.


.. py:exception:: Looking(*args, **kwargs)


   Bases: :py:obj:`Task`

   
   Create a new plan failure.


.. py:exception:: ObjectPoseMisestimation(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: SuccessfulCompletion(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: ObjectNotFound(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: LocomotorFailure(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: ArmFailure(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: ObjectLost(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: SensorFailure(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: IllPosedGoalFailure(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: CapabilityAbsenceFailure(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: ReachabilityFailure(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: TorsoFailure(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: ConfigurationNotReached(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: Timeout(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: EndEffectorFailure(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: ObjectUnavailable(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: SustainedFailure(*args, **kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: ReasoningError(**kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


.. py:exception:: CollisionError(**kwargs)


   Bases: :py:obj:`PlanFailure`

   Implementation of plan failures.

   Create a new plan failure.


