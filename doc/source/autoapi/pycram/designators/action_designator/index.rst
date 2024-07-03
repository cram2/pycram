:py:mod:`pycram.designators.action_designator`
==============================================

.. py:module:: pycram.designators.action_designator


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.designators.action_designator.MoveTorsoAction
   pycram.designators.action_designator.SetGripperAction
   pycram.designators.action_designator.ReleaseAction
   pycram.designators.action_designator.GripAction
   pycram.designators.action_designator.ParkArmsAction
   pycram.designators.action_designator.PickUpAction
   pycram.designators.action_designator.PlaceAction
   pycram.designators.action_designator.NavigateAction
   pycram.designators.action_designator.TransportAction
   pycram.designators.action_designator.LookAtAction
   pycram.designators.action_designator.DetectAction
   pycram.designators.action_designator.OpenAction
   pycram.designators.action_designator.CloseAction
   pycram.designators.action_designator.GraspingAction
   pycram.designators.action_designator.ActionAbstract
   pycram.designators.action_designator.MoveTorsoActionPerformable
   pycram.designators.action_designator.SetGripperActionPerformable
   pycram.designators.action_designator.ReleaseActionPerformable
   pycram.designators.action_designator.GripActionPerformable
   pycram.designators.action_designator.ParkArmsActionPerformable
   pycram.designators.action_designator.PickUpActionPerformable
   pycram.designators.action_designator.PlaceActionPerformable
   pycram.designators.action_designator.NavigateActionPerformable
   pycram.designators.action_designator.TransportActionPerformable
   pycram.designators.action_designator.LookAtActionPerformable
   pycram.designators.action_designator.DetectActionPerformable
   pycram.designators.action_designator.OpenActionPerformable
   pycram.designators.action_designator.CloseActionPerformable
   pycram.designators.action_designator.GraspingActionPerformable
   pycram.designators.action_designator.FaceAtPerformable
   pycram.designators.action_designator.MoveAndPickUpPerformable




.. py:class:: MoveTorsoAction(positions: typing_extensions.List[float], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[pycram.ontology.ontology.OntologyConceptHolder]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Action Designator for Moving the torso of the robot up and down

   Create a designator description to move the torso of the robot up and down.

   :param positions: List of possible positions of the robots torso, possible position is a float of height in metres
   :param resolver: An optional specialized_designators that returns a performable designator for a designator description.
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> MoveTorsoActionPerformable

      Creates a performable action designator with the first element from the list of possible torso heights.

      :return: A performable action designator


   .. py:method:: __iter__()

      Iterates over all possible values for this designator and returns a performable action designator with the value.

      :return: A performable action designator



.. py:class:: SetGripperAction(grippers: typing_extensions.List[pycram.datastructures.enums.Arms], motions: typing_extensions.List[pycram.datastructures.enums.GripperState], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Set the gripper state of the robot

   Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

   :param grippers: A list of possible grippers
   :param motions: A list of possible motions
   :param resolver: An alternative specialized_designators that returns a performable designator for a designator description
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> SetGripperActionPerformable

      Default specialized_designators that returns a performable designator with the first element in the grippers and motions list.

      :return: A performable designator


   .. py:method:: __iter__()

      Iterates over all possible combinations of grippers and motions

      :return: A performable designator with a combination of gripper and motion



.. py:class:: ReleaseAction(grippers: typing_extensions.List[pycram.datastructures.enums.Arms], object_designator_description: pycram.designators.object_designator.ObjectDesignatorDescription, resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Releases an Object from the robot.

   Note: This action can not be used yet.

   Base of all action designator descriptions.

   :param resolver: An alternative resolver that returns an action designator
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> ReleaseActionPerformable

      Fill all missing parameters and chose plan to execute.



.. py:class:: GripAction(grippers: typing_extensions.List[pycram.datastructures.enums.Arms], object_designator_description: pycram.designators.object_designator.ObjectDesignatorDescription, efforts: typing_extensions.List[float], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Grip an object with the robot.

   :ivar grippers: The grippers to consider
   :ivar object_designator_description: The description of objects to consider
   :ivar efforts: The efforts to consider

   Note: This action can not be used yet.

   Base of all action designator descriptions.

   :param resolver: An alternative resolver that returns an action designator
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> GripActionPerformable

      Fill all missing parameters and chose plan to execute.



.. py:class:: ParkArmsAction(arms: typing_extensions.List[pycram.datastructures.enums.Arms], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Park the arms of the robot.

   Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

   :param arms: A list of possible arms, that could be used
   :param resolver: An optional specialized_designators that returns a performable designator from the designator description
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> ParkArmsActionPerformable

      Default specialized_designators that returns a performable designator with the first element of the list of possible arms

      :return: A performable designator



.. py:class:: PickUpAction(object_designator_description: typing_extensions.Union[pycram.designators.object_designator.ObjectDesignatorDescription, pycram.designators.object_designator.ObjectDesignatorDescription.Object], arms: typing_extensions.List[pycram.datastructures.enums.Arms], grasps: typing_extensions.List[pycram.datastructures.enums.Grasp], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Designator to let the robot pick up an object.

   Lets the robot pick up an object. The description needs an object designator describing the object that should be
   picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

   :param object_designator_description: List of possible object designator
   :param arms: List of possible arms that could be used
   :param grasps: List of possible grasps for the object
   :param resolver: An optional specialized_designators that returns a performable designator with elements from the lists of possible paramter
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> PickUpActionPerformable

      Default specialized_designators, returns a performable designator with the first entries from the lists of possible parameter.

      :return: A performable designator



.. py:class:: PlaceAction(object_designator_description: typing_extensions.Union[pycram.designators.object_designator.ObjectDesignatorDescription, pycram.designators.object_designator.ObjectDesignatorDescription.Object], target_locations: typing_extensions.List[pycram.datastructures.pose.Pose], arms: typing_extensions.List[pycram.datastructures.enums.Arms], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Places an Object at a position using an arm.

   Create an Action Description to place an object

   :param object_designator_description: Description of object to place.
   :param target_locations: List of possible positions/orientations to place the object
   :param arms: List of possible arms to use
   :param resolver: Grounding method to resolve this designator
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> PlaceActionPerformable

      Default specialized_designators that returns a performable designator with the first entries from the list of possible entries.

      :return: A performable designator



.. py:class:: NavigateAction(target_locations: typing_extensions.List[pycram.datastructures.pose.Pose], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Navigates the Robot to a position.

   Navigates the robot to a location.

   :param target_locations: A list of possible target locations for the navigation.
   :param resolver: An alternative specialized_designators that creates a performable designator from the list of possible parameter
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> NavigateActionPerformable

      Default specialized_designators that returns a performable designator with the first entry of possible target locations.

      :return: A performable designator



.. py:class:: TransportAction(object_designator_description: typing_extensions.Union[pycram.designators.object_designator.ObjectDesignatorDescription, pycram.designators.object_designator.ObjectDesignatorDescription.Object], arms: typing_extensions.List[pycram.datastructures.enums.Arms], target_locations: typing_extensions.List[pycram.datastructures.pose.Pose], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Transports an object to a position using an arm

   Designator representing a pick and place plan.

   :param object_designator_description: Object designator description or a specified Object designator that should be transported
   :param arms: A List of possible arms that could be used for transporting
   :param target_locations: A list of possible target locations for the object to be placed
   :param resolver: An alternative specialized_designators that returns a performable designator for the list of possible parameter
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> TransportActionPerformable

      Default specialized_designators that returns a performable designator with the first entries from the lists of possible parameter.

      :return: A performable designator



.. py:class:: LookAtAction(targets: typing_extensions.List[pycram.datastructures.pose.Pose], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Lets the robot look at a position.

   Moves the head of the robot such that it points towards the given target location.

   :param targets: A list of possible locations to look at
   :param resolver: An alternative specialized_designators that returns a performable designator for a list of possible target locations
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> LookAtActionPerformable

      Default specialized_designators that returns a performable designator with the first entry in the list of possible targets

      :return: A performable designator



.. py:class:: DetectAction(object_designator_description: pycram.designators.object_designator.ObjectDesignatorDescription, resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Detects an object that fits the object description and returns an object designator describing the object.

   Tries to detect an object in the field of view (FOV) of the robot.

   :param object_designator_description: Object designator describing the object
   :param resolver: An alternative specialized_designators
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> DetectActionPerformable

      Default specialized_designators that returns a performable designator with the resolved object description.

      :return: A performable designator



.. py:class:: OpenAction(object_designator_description: pycram.designators.object_designator.ObjectPart, arms: typing_extensions.List[pycram.datastructures.enums.Arms], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Opens a container like object

   Can currently not be used

   Moves the arm of the robot to open a container.

   :param object_designator_description: Object designator describing the handle that should be used to open
   :param arms: A list of possible arms that should be used
   :param resolver: A alternative specialized_designators that returns a performable designator for the lists of possible parameter.
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> OpenActionPerformable

      Default specialized_designators that returns a performable designator with the resolved object description and the first entries
      from the lists of possible parameter.

      :return: A performable designator



.. py:class:: CloseAction(object_designator_description: pycram.designators.object_designator.ObjectPart, arms: typing_extensions.List[pycram.datastructures.enums.Arms], resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Closes a container like object.

   Can currently not be used

   Attempts to close an open container

   :param object_designator_description: Object designator description of the handle that should be used
   :param arms: A list of possible arms to use
   :param resolver: An alternative specialized_designators that returns a performable designator for the list of possible parameter
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> CloseActionPerformable

      Default specialized_designators that returns a performable designator with the resolved object designator and the first entry from
      the list of possible arms.

      :return: A performable designator



.. py:class:: GraspingAction(arms: typing_extensions.List[pycram.datastructures.enums.Arms], object_description: typing_extensions.Union[pycram.designators.object_designator.ObjectDesignatorDescription, pycram.designators.object_designator.ObjectPart], resolver: typing_extensions.Callable = None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription`

   Grasps an object described by the given Object Designator description

   Will try to grasp the object described by the given description. Grasping is done by moving into a pre grasp
   position 10 cm before the object, opening the gripper, moving to the object and then closing the gripper.

   :param arms: List of Arms that should be used for grasping
   :param object_description: Description of the object that should be grasped
   :param resolver: An alternative specialized_designators to get a specified designator from the designator description
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:method:: ground() -> GraspingActionPerformable

      Default specialized_designators that takes the first element from the list of arms and the first solution for the object
      designator description ond returns it.

      :return: A performable action designator that contains specific arguments



.. py:class:: ActionAbstract


   Bases: :py:obj:`pycram.designator.ActionDesignatorDescription.Action`, :py:obj:`abc.ABC`

   Base class for performable performables.

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[pycram.orm.action_designator.Action]

      The ORM class that is used to insert this action into the database. Must be overwritten by every action in order to
      be able to insert the action into the database.

   .. py:method:: perform() -> None
      :abstractmethod:

      Perform the action.

      Will be overwritten by each action.


   .. py:method:: to_sql() -> pycram.orm.action_designator.Action

      Convert this action to its ORM equivalent.

      Needs to be overwritten by an action if it didn't overwrite the orm_class attribute with its ORM equivalent.

      :return: An instance of the ORM equivalent of the action with the parameters set


   .. py:method:: insert(session: sqlalchemy.orm.Session, **kwargs) -> pycram.orm.action_designator.Action

      Insert this action into the database.

      Needs to be overwritten by an action if the action has attributes that do not exist in the orm class
      equivalent. In that case, the attributes need to be inserted into the session manually.

      :param session: Session with a database that is used to add and commit the objects
      :param kwargs: Possible extra keyword arguments
      :return: The completely instanced ORM action that was inserted into the database



.. py:class:: MoveTorsoActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Move the torso of the robot up and down.

   .. py:attribute:: position
      :type: float

      Target position of the torso joint

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: SetGripperActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Set the gripper state of the robot.

   .. py:attribute:: gripper
      :type: pycram.datastructures.enums.Arms

      The gripper that should be set

   .. py:attribute:: motion
      :type: pycram.datastructures.enums.GripperState

      The motion that should be set on the gripper

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: ReleaseActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Releases an Object from the robot.

   Note: This action can not ve used yet.

   .. py:attribute:: gripper
      :type: pycram.datastructures.enums.Arms

      

   .. py:attribute:: object_designator
      :type: pycram.designators.object_designator.ObjectDesignatorDescription.Object

      

   .. py:method:: perform() -> None
      :abstractmethod:

      Perform the action.

      Will be overwritten by each action.



.. py:class:: GripActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Grip an object with the robot.

   Note: This action can not be used yet.

   .. py:attribute:: gripper
      :type: pycram.datastructures.enums.Arms

      

   .. py:attribute:: object_designator
      :type: pycram.designators.object_designator.ObjectDesignatorDescription.Object

      

   .. py:attribute:: effort
      :type: float

      

   .. py:method:: perform() -> None
      :abstractmethod:

      Perform the action.

      Will be overwritten by each action.



.. py:class:: ParkArmsActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Park the arms of the robot.

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      Entry from the enum for which arm should be parked

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: PickUpActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Let the robot pick up an object.

   .. py:attribute:: object_designator
      :type: pycram.designators.object_designator.ObjectDesignatorDescription.Object

      Object designator describing the object that should be picked up

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      The arm that should be used for pick up

   .. py:attribute:: grasp
      :type: pycram.datastructures.enums.Grasp

      The grasp that should be used. For example, 'left' or 'right'

   .. py:attribute:: object_at_execution
      :type: typing_extensions.Optional[pycram.designators.object_designator.ObjectDesignatorDescription.Object]

      The object at the time this Action got created. It is used to be a static, information holding entity. It is
      not updated when the BulletWorld object is changed.

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: PlaceActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Places an Object at a position using an arm.

   .. py:attribute:: object_designator
      :type: pycram.designators.object_designator.ObjectDesignatorDescription.Object

      Object designator describing the object that should be place

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      Arm that is currently holding the object

   .. py:attribute:: target_location
      :type: pycram.datastructures.pose.Pose

      Pose in the world at which the object should be placed

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: NavigateActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Navigates the Robot to a position.

   .. py:attribute:: target_location
      :type: pycram.datastructures.pose.Pose

      Location to which the robot should be navigated

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: TransportActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Transports an object to a position using an arm

   .. py:attribute:: object_designator
      :type: pycram.designators.object_designator.ObjectDesignatorDescription.Object

      Object designator describing the object that should be transported.

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      Arm that should be used

   .. py:attribute:: target_location
      :type: pycram.datastructures.pose.Pose

      Target Location to which the object should be transported

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: LookAtActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Lets the robot look at a position.

   .. py:attribute:: target
      :type: pycram.datastructures.pose.Pose

      Position at which the robot should look, given as 6D pose

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: DetectActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Detects an object that fits the object description and returns an object designator describing the object.

   .. py:attribute:: object_designator
      :type: pycram.designators.object_designator.ObjectDesignatorDescription.Object

      Object designator loosely describing the object, e.g. only type.

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: OpenActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Opens a container like object

   .. py:attribute:: object_designator
      :type: pycram.designators.object_designator.ObjectPart.Object

      Object designator describing the object that should be opened

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      Arm that should be used for opening the container

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: CloseActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Closes a container like object.

   .. py:attribute:: object_designator
      :type: pycram.designators.object_designator.ObjectPart.Object

      Object designator describing the object that should be closed

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      Arm that should be used for closing

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: GraspingActionPerformable


   Bases: :py:obj:`ActionAbstract`

   Grasps an object described by the given Object Designator description

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      The arm that should be used to grasp

   .. py:attribute:: object_desig
      :type: typing_extensions.Union[pycram.designators.object_designator.ObjectDesignatorDescription.Object, pycram.designators.object_designator.ObjectPart.Object]

      Object Designator for the object that should be grasped

   .. py:attribute:: orm_class
      :type: typing_extensions.Type[ActionAbstract]

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: FaceAtPerformable


   Bases: :py:obj:`ActionAbstract`

   Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.

   .. py:attribute:: pose
      :type: pycram.datastructures.pose.Pose

      The pose to face

   .. py:attribute:: orm_class

      

   .. py:method:: perform() -> None

      Perform the action.

      Will be overwritten by each action.



.. py:class:: MoveAndPickUpPerformable


   Bases: :py:obj:`ActionAbstract`

   Navigate to `standing_position`, then turn towards the object and pick it up.

   .. py:attribute:: standing_position
      :type: pycram.datastructures.pose.Pose

      The pose to stand before trying to pick up the object

   .. py:attribute:: object_designator
      :type: pycram.designators.object_designator.ObjectDesignatorDescription.Object

      The object to pick up

   .. py:attribute:: arm
      :type: pycram.datastructures.enums.Arms

      The arm to use

   .. py:attribute:: grasp
      :type: pycram.datastructures.enums.Grasp

      The grasp to use

   .. py:method:: perform()

      Perform the action.

      Will be overwritten by each action.



