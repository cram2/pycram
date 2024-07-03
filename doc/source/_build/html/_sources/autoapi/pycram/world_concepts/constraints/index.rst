:py:mod:`pycram.world_concepts.constraints`
===========================================

.. py:module:: pycram.world_concepts.constraints


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.world_concepts.constraints.AbstractConstraint
   pycram.world_concepts.constraints.Constraint
   pycram.world_concepts.constraints.Attachment




.. py:class:: AbstractConstraint(parent_link: pycram.description.Link, child_link: pycram.description.Link, _type: pycram.datastructures.enums.JointType, parent_to_constraint: pycram.datastructures.pose.Transform, child_to_constraint: pycram.datastructures.pose.Transform)


   Represents an abstract constraint concept, this could be used to create joints for example or any kind of constraint
   between two links in the world.

   .. py:property:: parent_to_child_transform
      :type: typing_extensions.Union[pycram.datastructures.pose.Transform, None]


   .. py:property:: parent_object_id
      :type: int

      Returns the id of the parent object of the constraint.

      :return: The id of the parent object of the constraint

   .. py:property:: child_object_id
      :type: int

      Returns the id of the child object of the constraint.

      :return: The id of the child object of the constraint

   .. py:property:: parent_link_id
      :type: int

      Returns the id of the parent link of the constraint.

      :return: The id of the parent link of the constraint

   .. py:property:: child_link_id
      :type: int

      Returns the id of the child link of the constraint.

      :return: The id of the child link of the constraint

   .. py:property:: position_wrt_parent_as_list
      :type: typing_extensions.List[float]

      Returns the constraint frame pose with respect to the parent origin as a list.

      :return: The constraint frame pose with respect to the parent origin as a list

   .. py:property:: orientation_wrt_parent_as_list
      :type: typing_extensions.List[float]

      Returns the constraint frame orientation with respect to the parent origin as a list.

      :return: The constraint frame orientation with respect to the parent origin as a list

   .. py:property:: pose_wrt_parent
      :type: pycram.datastructures.pose.Pose

      Returns the joint frame pose with respect to the parent origin.

      :return: The joint frame pose with respect to the parent origin

   .. py:property:: position_wrt_child_as_list
      :type: typing_extensions.List[float]

      Returns the constraint frame pose with respect to the child origin as a list.

      :return: The constraint frame pose with respect to the child origin as a list

   .. py:property:: orientation_wrt_child_as_list
      :type: typing_extensions.List[float]

      Returns the constraint frame orientation with respect to the child origin as a list.

      :return: The constraint frame orientation with respect to the child origin as a list

   .. py:property:: pose_wrt_child
      :type: pycram.datastructures.pose.Pose

      Returns the joint frame pose with respect to the child origin.

      :return: The joint frame pose with respect to the child origin


.. py:class:: Constraint(parent_link: pycram.description.Link, child_link: pycram.description.Link, _type: pycram.datastructures.enums.JointType, axis_in_child_frame: geometry_msgs.msg.Point, constraint_to_parent: pycram.datastructures.pose.Transform, child_to_constraint: pycram.datastructures.pose.Transform)


   Bases: :py:obj:`AbstractConstraint`

   Represents a constraint between two links in the World.

   .. py:property:: axis_as_list
      :type: typing_extensions.List[float]

      Returns the axis of this constraint as a list.

      :return: The axis of this constraint as a list of xyz


.. py:class:: Attachment(parent_link: pycram.description.Link, child_link: pycram.description.Link, bidirectional: typing_extensions.Optional[bool] = False, parent_to_child_transform: typing_extensions.Optional[pycram.datastructures.pose.Transform] = None, constraint_id: typing_extensions.Optional[int] = None)


   Bases: :py:obj:`AbstractConstraint`

   Represents an abstract constraint concept, this could be used to create joints for example or any kind of constraint
   between two links in the world.

   Creates an attachment between the parent object link and the child object link.
   This could be a bidirectional attachment, meaning that both objects will move when one moves.

   :param parent_link: The parent object link.
   :param child_link: The child object link.
   :param bidirectional: If true, both objects will move when one moves.
   :param parent_to_child_transform: The transform from the parent link to the child object link.
   :param constraint_id: The id of the constraint in the simulator.

   .. py:property:: loose
      :type: bool

      If true, then the child object will not move when parent moves.

   .. py:property:: is_reversed
      :type: bool

      True if the parent and child links are swapped.

      :type: return

   .. py:method:: update_transform_and_constraint() -> None

      Updates the transform and constraint of this attachment.


   .. py:method:: update_transform() -> None

      Updates the transform of this attachment by calculating the transform from the parent link to the child link.


   .. py:method:: update_constraint() -> None

      Updates the constraint of this attachment by removing the old constraint if one exists and adding a new one.


   .. py:method:: add_fixed_constraint() -> None

      Adds a fixed constraint between the parent link and the child link.


   .. py:method:: calculate_transform() -> pycram.datastructures.pose.Transform

      Calculates the transform from the parent link to the child link.


   .. py:method:: remove_constraint_if_exists() -> None

      Removes the constraint between the parent and the child links if one exists.


   .. py:method:: get_inverse() -> Attachment

      :return: A new Attachment object with the parent and child links swapped.


   .. py:method:: __del__() -> None

      Removes the constraint between the parent and the child links if one exists when the attachment is deleted.


   .. py:method:: __copy__()


   .. py:method:: __eq__(other)

      Return self==value.


   .. py:method:: __hash__()

      Return hash(self).



