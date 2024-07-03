:py:mod:`pycram.designators.specialized_designators.probabilistic.probabilistic_action`
=======================================================================================

.. py:module:: pycram.designators.specialized_designators.probabilistic.probabilistic_action


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.designators.specialized_designators.probabilistic.probabilistic_action.Grasp
   pycram.designators.specialized_designators.probabilistic.probabilistic_action.Arms
   pycram.designators.specialized_designators.probabilistic.probabilistic_action.ProbabilisticAction
   pycram.designators.specialized_designators.probabilistic.probabilistic_action.GaussianCostmapModel
   pycram.designators.specialized_designators.probabilistic.probabilistic_action.MoveAndPickUp




.. py:class:: Grasp


   Bases: :py:obj:`random_events.set.SetElement`

   Base class for enums that are used as elements in a set.

   Classes that inherit from this class have to define an attribute called EMPTY_SET.
   It is advisable to define EMPTY_SET as -1 to correctly work with indices.
   The empty set of the class is used to access all other elements of the class.

   Initialize self.  See help(type(self)) for accurate signature.

   .. py:attribute:: EMPTY_SET

      

   .. py:attribute:: FRONT
      :value: 0

      

   .. py:attribute:: LEFT
      :value: 1

      

   .. py:attribute:: RIGHT
      :value: 2

      

   .. py:attribute:: TOP
      :value: 3

      


.. py:class:: Arms


   Bases: :py:obj:`random_events.set.SetElement`

   Base class for enums that are used as elements in a set.

   Classes that inherit from this class have to define an attribute called EMPTY_SET.
   It is advisable to define EMPTY_SET as -1 to correctly work with indices.
   The empty set of the class is used to access all other elements of the class.

   Initialize self.  See help(type(self)) for accurate signature.

   .. py:attribute:: EMPTY_SET

      

   .. py:attribute:: LEFT
      :value: 0

      

   .. py:attribute:: RIGHT
      :value: 1

      


.. py:class:: ProbabilisticAction(policy: typing_extensions.Optional[probabilistic_model.probabilistic_circuit.probabilistic_circuit.ProbabilisticCircuit] = None)


   Abstract class for probabilistic performables.

   .. py:class:: Variables


      Variables for probabilistic performables.

      This inner class serves the purpose to define the variables that are used in a model and make them easily
      accessible for the user. The user can access the variables by using the dot notation, e.g. `self.variables.x`.

      The members of this class have to be ordered the same way as the variables in the policy.
      The order of the variables in the policy is most likely alphabetical by name.


   .. py:attribute:: policy
      :type: probabilistic_model.probabilistic_circuit.probabilistic_circuit.ProbabilisticCircuit

      The policy that is used to determine the parameters.

   .. py:attribute:: variables
      :type: ProbabilisticAction.Variables

      The variables of this action.

   .. py:method:: default_policy() -> probabilistic_model.probabilistic_circuit.probabilistic_circuit.ProbabilisticCircuit
      :abstractmethod:

      Create a default policy for the action.

      :return: The default policy for the action


   .. py:method:: sample_to_action(sample: typing_extensions.List) -> pycram.designators.action_designator.ActionAbstract
      :abstractmethod:

      Convert a sample from the policy to a performable action.
      :param sample: The sample
      :return: The action



.. py:class:: GaussianCostmapModel(distance_to_center: float = 0.2, variance: float = 0.5)


   Class that generates a Gaussian Costmap around the center of an object. The costmap cuts out a square in the middle
   that has side lengths given by ``distance_to_center``.

   .. py:attribute:: distance_to_center
      :type: float

      The side length of the cut out square.

   .. py:attribute:: variance
      :type: float

      The variance of the distributions involved

   .. py:attribute:: relative_x

      

   .. py:attribute:: relative_y

      

   .. py:attribute:: grasp

      

   .. py:attribute:: arm

      

   .. py:method:: center_event() -> random_events.product_algebra.Event

      Create an event that describes the center of the map.


   .. py:method:: create_model_with_center() -> probabilistic_model.probabilistic_circuit.probabilistic_circuit.ProbabilisticCircuit

      Create a fully factorized gaussian at the center of the map.


   .. py:method:: create_model() -> probabilistic_model.probabilistic_circuit.probabilistic_circuit.ProbabilisticCircuit

      Create a gaussian model that assumes mass everywhere besides the center square.

      :return: The probabilistic circuit



.. py:class:: MoveAndPickUp(object_designator: pycram.designator.ObjectDesignatorDescription.Object, arms: typing_extensions.List[Arms], grasps: typing_extensions.List[Grasp], policy: typing_extensions.Optional[probabilistic_model.probabilistic_circuit.probabilistic_circuit.ProbabilisticCircuit] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`, :py:obj:`ProbabilisticAction`

   Abstract class for action designator descriptions.
   Descriptions hold possible parameter ranges for action designators.

   Base of all action designator descriptions.

   :param resolver: An alternative resolver that returns an action designator
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:class:: Variables


      .. py:attribute:: arm
         :type: random_events.variable.Symbolic

         

      .. py:attribute:: grasp
         :type: random_events.variable.Symbolic

         

      .. py:attribute:: relative_x
         :type: random_events.variable.Continuous

         

      .. py:attribute:: relative_y
         :type: random_events.variable.Continuous

         


   .. py:attribute:: variables
      :type: MoveAndPickUp.Variables

      

   .. py:attribute:: sample_amount
      :type: int
      :value: 20

      The amount of samples that should be drawn from the policy when iterating over it.

   .. py:attribute:: object_designator
      :type: pycram.designator.ObjectDesignatorDescription.Object

      The object designator that should be picked up.

   .. py:attribute:: arms
      :type: typing_extensions.List[Arms]

      The arms that can be used for the pick up.

   .. py:attribute:: grasps
      :type: typing_extensions.List[Grasp]

      The grasps that can be used for the pick up.

   .. py:method:: default_policy() -> probabilistic_model.probabilistic_circuit.probabilistic_circuit.ProbabilisticCircuit

      Create a default policy for the action.

      :return: The default policy for the action


   .. py:method:: sample_to_action(sample: typing_extensions.List) -> pycram.designators.action_designator.MoveAndPickUpPerformable

      Convert a sample from the underlying distribution to a performable action.
      :param sample: The sample
      :return:  action


   .. py:method:: events_from_occupancy_and_visibility_costmap() -> random_events.product_algebra.Event

      Create events from the occupancy and visibility costmap.

      :return: The events that can be used as evidence for the model.


   .. py:method:: ground_model(model: typing_extensions.Optional[probabilistic_model.probabilistic_circuit.probabilistic_circuit.ProbabilisticCircuit] = None, event: typing_extensions.Optional[random_events.product_algebra.Event] = None) -> probabilistic_model.probabilistic_circuit.probabilistic_circuit.ProbabilisticCircuit

      Ground the model to the current evidence.

      :param model: The model that should be grounded. If None, the policy is used.
      :param event: The events that should be used as evidence. If None, the occupancy costmap is used.
      :return: The grounded model


   .. py:method:: iter_with_mode() -> typing_extensions.Iterator[pycram.designators.action_designator.MoveAndPickUpPerformable]

      Generate performables by sampling from the mode of the policy conditioned on visibility and occupancy.


   .. py:method:: __iter__() -> typing_extensions.Iterator[pycram.designators.action_designator.MoveAndPickUpPerformable]

      Generate performables by sampling from the policy conditioned on visibility and occupancy.


   .. py:method:: iterate_without_occupancy_costmap() -> typing_extensions.Iterator[pycram.designators.action_designator.MoveAndPickUpPerformable]

      Generate performables by sampling from the policy without conditioning on visibility and occupancy.


   .. py:method:: query_for_database()
      :staticmethod:


   .. py:method:: batch_rollout()

      Try the policy without conditioning on visibility and occupancy and count the successful tries.

      :amount: The amount of tries



