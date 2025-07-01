from ormatic.custom_types import TypeType
from sqlalchemy import Boolean, Column, DateTime, Enum, Float, ForeignKey, Integer, MetaData, String, Table
from sqlalchemy.orm import RelationshipProperty, registry, relationship
import pycram.datastructures.dataclasses
import pycram.datastructures.grasp
import pycram.datastructures.pose
import pycram.designator
import pycram.designators.action_designator
import pycram.language
import pycram.orm.model
import pycram.plan

metadata = MetaData()


t_CodeNode = Table(
    'CodeNode', metadata,
    Column('id', Integer, primary_key=True)
)

t_DesignatorNode = Table(
    'DesignatorNode', metadata,
    Column('id', Integer, primary_key=True),
    Column('status', Enum(pycram.datastructures.enums.TaskStatus), nullable=False),
    Column('start_time', DateTime),
    Column('end_time', DateTime),
    Column('polymorphic_type', String(255))
)

t_GraspDescription = Table(
    'GraspDescription', metadata,
    Column('id', Integer, primary_key=True),
    Column('approach_direction', Enum(pycram.datastructures.enums.Grasp), nullable=False),
    Column('vertical_alignment', Enum(pycram.datastructures.enums.Grasp)),
    Column('rotate_gripper', Boolean, nullable=False)
)

t_Header = Table(
    'Header', metadata,
    Column('id', Integer, primary_key=True),
    Column('frame_id', String(255), nullable=False),
    Column('stamp', DateTime, nullable=False),
    Column('sequence', Integer, nullable=False)
)

t_MonitorNode = Table(
    'MonitorNode', metadata,
    Column('id', Integer, primary_key=True)
)

t_MotionNode = Table(
    'MotionNode', metadata,
    Column('id', Integer, primary_key=True),
    Column('status', Enum(pycram.datastructures.enums.TaskStatus), nullable=False),
    Column('start_time', DateTime),
    Column('end_time', DateTime)
)

t_ParallelNode = Table(
    'ParallelNode', metadata,
    Column('id', Integer, primary_key=True)
)

t_PlanNode = Table(
    'PlanNode', metadata,
    Column('id', Integer, primary_key=True),
    Column('status', Enum(pycram.datastructures.enums.TaskStatus), nullable=False),
    Column('start_time', DateTime),
    Column('end_time', DateTime)
)

t_PreferredGraspAlignment = Table(
    'PreferredGraspAlignment', metadata,
    Column('id', Integer, primary_key=True),
    Column('preferred_axis', Enum(pycram.datastructures.enums.AxisIdentifier)),
    Column('with_vertical_alignment', Boolean, nullable=False),
    Column('with_rotated_gripper', Boolean, nullable=False)
)

t_Quaternion = Table(
    'Quaternion', metadata,
    Column('id', Integer, primary_key=True),
    Column('x', Float, nullable=False),
    Column('y', Float, nullable=False),
    Column('z', Float, nullable=False),
    Column('w', Float, nullable=False)
)

t_SequentialNode = Table(
    'SequentialNode', metadata,
    Column('id', Integer, primary_key=True),
    Column('status', Enum(pycram.datastructures.enums.TaskStatus), nullable=False),
    Column('start_time', DateTime),
    Column('end_time', DateTime),
    Column('action', TypeType),
    Column('polymorphic_type', String(255))
)

t_TryAllNode = Table(
    'TryAllNode', metadata,
    Column('id', Integer, primary_key=True)
)

t_TryInOrderNode = Table(
    'TryInOrderNode', metadata,
    Column('id', Integer, primary_key=True)
)

t_Vector3 = Table(
    'Vector3', metadata,
    Column('id', Integer, primary_key=True),
    Column('x', Float, nullable=False),
    Column('y', Float, nullable=False),
    Column('z', Float, nullable=False),
    Column('polymorphic_type', String(255))
)

t_ActionNode = Table(
    'ActionNode', metadata,
    Column('id', ForeignKey('DesignatorNode.id'), primary_key=True),
    Column('status', Enum(pycram.datastructures.enums.TaskStatus), nullable=False),
    Column('start_time', DateTime),
    Column('end_time', DateTime)
)

t_Pose = Table(
    'Pose', metadata,
    Column('id', Integer, primary_key=True),
    Column('position_id', ForeignKey('Vector3.id'), nullable=False),
    Column('orientation_id', ForeignKey('Quaternion.id'), nullable=False),
    Column('polymorphic_type', String(255))
)

t_RepeatNode = Table(
    'RepeatNode', metadata,
    Column('id', ForeignKey('SequentialNode.id'), primary_key=True),
    Column('repeat', Integer, nullable=False)
)

t_Vector3Stamped = Table(
    'Vector3Stamped', metadata,
    Column('id', ForeignKey('Vector3.id'), primary_key=True),
    Column('header_id', ForeignKey('Header.id'), nullable=False)
)

t_PoseStamped = Table(
    'PoseStamped', metadata,
    Column('id', Integer, primary_key=True),
    Column('pose_id', ForeignKey('Pose.id'), nullable=False),
    Column('header_id', ForeignKey('Header.id'), nullable=False),
    Column('polymorphic_type', String(255))
)

t_Transform = Table(
    'Transform', metadata,
    Column('id', ForeignKey('Pose.id'), primary_key=True)
)

t_ActionDescription = Table(
    'ActionDescription', metadata,
    Column('id', Integer, primary_key=True),
    Column('robot_position_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('robot_torso_height', Float, nullable=False),
    Column('robot_type', TypeType),
    Column('polymorphic_type', String(255))
)

t_FrozenObject = Table(
    'FrozenObject', metadata,
    Column('id', Integer, primary_key=True),
    Column('name', String(255), nullable=False),
    Column('concept', TypeType),
    Column('pose_id', ForeignKey('PoseStamped.id'))
)

t_GraspPose = Table(
    'GraspPose', metadata,
    Column('id', ForeignKey('PoseStamped.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('grasp_description_id', ForeignKey('GraspDescription.id'), nullable=False)
)

t_TransformStamped = Table(
    'TransformStamped', metadata,
    Column('id', ForeignKey('PoseStamped.id'), primary_key=True),
    Column('pose_id', ForeignKey('Transform.id'), nullable=False),
    Column('child_frame_id', String(255), nullable=False)
)

t_CarryAction = Table(
    'CarryAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('align', Boolean),
    Column('tip_link', String(255)),
    Column('tip_axis', Enum(pycram.datastructures.enums.AxisIdentifier)),
    Column('root_link', String(255)),
    Column('root_axis', Enum(pycram.datastructures.enums.AxisIdentifier))
)

t_CloseAction = Table(
    'CloseAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('grasping_prepose_distance', Float, nullable=False)
)

t_DetectAction = Table(
    'DetectAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('technique', Enum(pycram.datastructures.enums.DetectionTechnique), nullable=False),
    Column('state', Enum(pycram.datastructures.enums.DetectionState))
)

t_FaceAtAction = Table(
    'FaceAtAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('pose_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('keep_joint_states', Boolean, nullable=False)
)

t_GraspingAction = Table(
    'GraspingAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('prepose_distance', Float, nullable=False)
)

t_GripAction = Table(
    'GripAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('object_at_execution_id', ForeignKey('FrozenObject.id')),
    Column('gripper', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('effort', Float, nullable=False)
)

t_LookAtAction = Table(
    'LookAtAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('target_id', ForeignKey('PoseStamped.id'), nullable=False)
)

t_MoveAndPickUpAction = Table(
    'MoveAndPickUpAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('standing_position_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('grasp_description_id', ForeignKey('GraspDescription.id'), nullable=False),
    Column('keep_joint_states', Boolean, nullable=False),
    Column('object_at_execution_id', ForeignKey('FrozenObject.id'))
)

t_MoveAndPlaceAction = Table(
    'MoveAndPlaceAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('standing_position_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('target_location_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('keep_joint_states', Boolean, nullable=False)
)

t_MoveTorsoAction = Table(
    'MoveTorsoAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('torso_state', Enum(pycram.datastructures.enums.TorsoState), nullable=False)
)

t_NavigateAction = Table(
    'NavigateAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('target_location_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('keep_joint_states', Boolean, nullable=False)
)

t_OpenAction = Table(
    'OpenAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('grasping_prepose_distance', Float, nullable=False)
)

t_ParkArmsAction = Table(
    'ParkArmsAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False)
)

t_PickAndPlaceAction = Table(
    'PickAndPlaceAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('target_location_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('grasp_description_id', ForeignKey('GraspDescription.id'), nullable=False)
)

t_PickUpAction = Table(
    'PickUpAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('grasp_description_id', ForeignKey('GraspDescription.id'), nullable=False),
    Column('object_at_execution_id', ForeignKey('FrozenObject.id'))
)

t_PlaceAction = Table(
    'PlaceAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('target_location_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('object_at_execution_id', ForeignKey('FrozenObject.id'))
)

t_ReachToPickUpAction = Table(
    'ReachToPickUpAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('grasp_description_id', ForeignKey('GraspDescription.id'), nullable=False),
    Column('object_at_execution_id', ForeignKey('FrozenObject.id'))
)

t_ReleaseAction = Table(
    'ReleaseAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('object_at_execution_id', ForeignKey('FrozenObject.id')),
    Column('gripper', Enum(pycram.datastructures.enums.Arms), nullable=False)
)

t_ResolvedActionNode = Table(
    'ResolvedActionNode', metadata,
    Column('id', Integer, primary_key=True),
    Column('designator_ref_id', ForeignKey('ActionDescription.id'), nullable=False),
    Column('status', Enum(pycram.datastructures.enums.TaskStatus), nullable=False),
    Column('start_time', DateTime),
    Column('end_time', DateTime)
)

t_SearchAction = Table(
    'SearchAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('target_location_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('object_type', TypeType)
)

t_SetGripperAction = Table(
    'SetGripperAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('gripper', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('motion', Enum(pycram.datastructures.enums.GripperState), nullable=False)
)

t_TransportAction = Table(
    'TransportAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('target_location_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('object_at_execution_id', ForeignKey('FrozenObject.id'))
)

mapper_registry = registry(metadata=metadata)

m_ActionDescription = mapper_registry.map_imperatively(pycram.designator.ActionDescription, t_ActionDescription, properties = dict(robot_position=relationship('PoseStamped',foreign_keys=[t_ActionDescription.c.robot_position_id]), 
robot_type=t_ActionDescription.c.robot_type), polymorphic_on = "polymorphic_type", polymorphic_identity = "ActionDescription")

m_DesignatorNode = mapper_registry.map_imperatively(pycram.plan.DesignatorNode, t_DesignatorNode, polymorphic_on = "polymorphic_type", polymorphic_identity = "DesignatorNode")

m_FrozenObject = mapper_registry.map_imperatively(pycram.datastructures.dataclasses.FrozenObject, t_FrozenObject, properties = dict(pose=relationship('PoseStamped',foreign_keys=[t_FrozenObject.c.pose_id]), 
concept=t_FrozenObject.c.concept))

m_SequentialNode = mapper_registry.map_imperatively(pycram.language.SequentialNode, t_SequentialNode, properties = dict(action=t_SequentialNode.c.action), polymorphic_on = "polymorphic_type", polymorphic_identity = "SequentialNode")

m_PoseStamped = mapper_registry.map_imperatively(pycram.datastructures.pose.PoseStamped, t_PoseStamped, properties = dict(pose=relationship('Pose',foreign_keys=[t_PoseStamped.c.pose_id]), 
header=relationship('Header',foreign_keys=[t_PoseStamped.c.header_id])), polymorphic_on = "polymorphic_type", polymorphic_identity = "PoseStamped")

m_ParallelNode = mapper_registry.map_imperatively(pycram.language.ParallelNode, t_ParallelNode, )

m_PlanNode = mapper_registry.map_imperatively(pycram.plan.PlanNode, t_PlanNode, )

m_CodeNode = mapper_registry.map_imperatively(pycram.language.CodeNode, t_CodeNode, )

m_TryAllNode = mapper_registry.map_imperatively(pycram.language.TryAllNode, t_TryAllNode, )

m_Pose = mapper_registry.map_imperatively(pycram.datastructures.pose.Pose, t_Pose, properties = dict(position=relationship('Vector3',foreign_keys=[t_Pose.c.position_id]), 
orientation=relationship('Quaternion',foreign_keys=[t_Pose.c.orientation_id])), polymorphic_on = "polymorphic_type", polymorphic_identity = "Pose")

m_PreferredGraspAlignment = mapper_registry.map_imperatively(pycram.datastructures.grasp.PreferredGraspAlignment, t_PreferredGraspAlignment, )

m_Header = mapper_registry.map_imperatively(pycram.datastructures.pose.Header, t_Header, )

m_Quaternion = mapper_registry.map_imperatively(pycram.datastructures.pose.Quaternion, t_Quaternion, )

m_TryInOrderNode = mapper_registry.map_imperatively(pycram.language.TryInOrderNode, t_TryInOrderNode, )

m_ResolvedActionNode = mapper_registry.map_imperatively(pycram.plan.ResolvedActionNode, t_ResolvedActionNode, properties = dict(designator_ref=relationship('ActionDescription',foreign_keys=[t_ResolvedActionNode.c.designator_ref_id])))

m_Vector3 = mapper_registry.map_imperatively(pycram.datastructures.pose.Vector3, t_Vector3, polymorphic_on = "polymorphic_type", polymorphic_identity = "Vector3")

m_MotionNode = mapper_registry.map_imperatively(pycram.plan.MotionNode, t_MotionNode, )

m_MonitorNode = mapper_registry.map_imperatively(pycram.language.MonitorNode, t_MonitorNode, )

m_GraspDescription = mapper_registry.map_imperatively(pycram.datastructures.grasp.GraspDescription, t_GraspDescription, )

m_CloseAction = mapper_registry.map_imperatively(pycram.designators.action_designator.CloseAction, t_CloseAction, polymorphic_identity = "CloseAction", inherits = m_ActionDescription)

m_GripAction = mapper_registry.map_imperatively(pycram.designators.action_designator.GripAction, t_GripAction, properties = dict(object_at_execution=relationship('FrozenObject',foreign_keys=[t_GripAction.c.object_at_execution_id])), polymorphic_identity = "GripAction", inherits = m_ActionDescription)

m_CarryAction = mapper_registry.map_imperatively(pycram.designators.action_designator.CarryAction, t_CarryAction, polymorphic_identity = "CarryAction", inherits = m_ActionDescription)

m_ReleaseAction = mapper_registry.map_imperatively(pycram.designators.action_designator.ReleaseAction, t_ReleaseAction, properties = dict(object_at_execution=relationship('FrozenObject',foreign_keys=[t_ReleaseAction.c.object_at_execution_id])), polymorphic_identity = "ReleaseAction", inherits = m_ActionDescription)

m_MoveAndPickUpAction = mapper_registry.map_imperatively(pycram.designators.action_designator.MoveAndPickUpAction, t_MoveAndPickUpAction, properties = dict(standing_position=relationship('PoseStamped',foreign_keys=[t_MoveAndPickUpAction.c.standing_position_id]), 
grasp_description=relationship('GraspDescription',foreign_keys=[t_MoveAndPickUpAction.c.grasp_description_id]), 
object_at_execution=relationship('FrozenObject',foreign_keys=[t_MoveAndPickUpAction.c.object_at_execution_id])), polymorphic_identity = "MoveAndPickUpAction", inherits = m_ActionDescription)

m_PlaceAction = mapper_registry.map_imperatively(pycram.designators.action_designator.PlaceAction, t_PlaceAction, properties = dict(target_location=relationship('PoseStamped',foreign_keys=[t_PlaceAction.c.target_location_id]), 
object_at_execution=relationship('FrozenObject',foreign_keys=[t_PlaceAction.c.object_at_execution_id])), polymorphic_identity = "PlaceAction", inherits = m_ActionDescription)

m_DetectAction = mapper_registry.map_imperatively(pycram.designators.action_designator.DetectAction, t_DetectAction, polymorphic_identity = "DetectAction", inherits = m_ActionDescription)

m_ParkArmsAction = mapper_registry.map_imperatively(pycram.designators.action_designator.ParkArmsAction, t_ParkArmsAction, polymorphic_identity = "ParkArmsAction", inherits = m_ActionDescription)

m_FaceAtAction = mapper_registry.map_imperatively(pycram.designators.action_designator.FaceAtAction, t_FaceAtAction, properties = dict(pose=relationship('PoseStamped',foreign_keys=[t_FaceAtAction.c.pose_id])), polymorphic_identity = "FaceAtAction", inherits = m_ActionDescription)

m_MoveTorsoAction = mapper_registry.map_imperatively(pycram.designators.action_designator.MoveTorsoAction, t_MoveTorsoAction, polymorphic_identity = "MoveTorsoAction", inherits = m_ActionDescription)

m_TransportAction = mapper_registry.map_imperatively(pycram.designators.action_designator.TransportAction, t_TransportAction, properties = dict(target_location=relationship('PoseStamped',foreign_keys=[t_TransportAction.c.target_location_id]), 
object_at_execution=relationship('FrozenObject',foreign_keys=[t_TransportAction.c.object_at_execution_id])), polymorphic_identity = "TransportAction", inherits = m_ActionDescription)

m_OpenAction = mapper_registry.map_imperatively(pycram.designators.action_designator.OpenAction, t_OpenAction, polymorphic_identity = "OpenAction", inherits = m_ActionDescription)

m_LookAtAction = mapper_registry.map_imperatively(pycram.designators.action_designator.LookAtAction, t_LookAtAction, properties = dict(target=relationship('PoseStamped',foreign_keys=[t_LookAtAction.c.target_id])), polymorphic_identity = "LookAtAction", inherits = m_ActionDescription)

m_PickAndPlaceAction = mapper_registry.map_imperatively(pycram.designators.action_designator.PickAndPlaceAction, t_PickAndPlaceAction, properties = dict(target_location=relationship('PoseStamped',foreign_keys=[t_PickAndPlaceAction.c.target_location_id]), 
grasp_description=relationship('GraspDescription',foreign_keys=[t_PickAndPlaceAction.c.grasp_description_id])), polymorphic_identity = "PickAndPlaceAction", inherits = m_ActionDescription)

m_PickUpAction = mapper_registry.map_imperatively(pycram.designators.action_designator.PickUpAction, t_PickUpAction, properties = dict(grasp_description=relationship('GraspDescription',foreign_keys=[t_PickUpAction.c.grasp_description_id]), 
object_at_execution=relationship('FrozenObject',foreign_keys=[t_PickUpAction.c.object_at_execution_id])), polymorphic_identity = "PickUpAction", inherits = m_ActionDescription)

m_NavigateAction = mapper_registry.map_imperatively(pycram.designators.action_designator.NavigateAction, t_NavigateAction, properties = dict(target_location=relationship('PoseStamped',foreign_keys=[t_NavigateAction.c.target_location_id])), polymorphic_identity = "NavigateAction", inherits = m_ActionDescription)

m_GraspingAction = mapper_registry.map_imperatively(pycram.designators.action_designator.GraspingAction, t_GraspingAction, polymorphic_identity = "GraspingAction", inherits = m_ActionDescription)

m_ReachToPickUpAction = mapper_registry.map_imperatively(pycram.designators.action_designator.ReachToPickUpAction, t_ReachToPickUpAction, properties = dict(grasp_description=relationship('GraspDescription',foreign_keys=[t_ReachToPickUpAction.c.grasp_description_id]), 
object_at_execution=relationship('FrozenObject',foreign_keys=[t_ReachToPickUpAction.c.object_at_execution_id])), polymorphic_identity = "ReachToPickUpAction", inherits = m_ActionDescription)

m_SetGripperAction = mapper_registry.map_imperatively(pycram.designators.action_designator.SetGripperAction, t_SetGripperAction, polymorphic_identity = "SetGripperAction", inherits = m_ActionDescription)

m_MoveAndPlaceAction = mapper_registry.map_imperatively(pycram.designators.action_designator.MoveAndPlaceAction, t_MoveAndPlaceAction, properties = dict(standing_position=relationship('PoseStamped',foreign_keys=[t_MoveAndPlaceAction.c.standing_position_id]), 
target_location=relationship('PoseStamped',foreign_keys=[t_MoveAndPlaceAction.c.target_location_id])), polymorphic_identity = "MoveAndPlaceAction", inherits = m_ActionDescription)

m_SearchAction = mapper_registry.map_imperatively(pycram.designators.action_designator.SearchAction, t_SearchAction, properties = dict(target_location=relationship('PoseStamped',foreign_keys=[t_SearchAction.c.target_location_id]), 
object_type=t_SearchAction.c.object_type), polymorphic_identity = "SearchAction", inherits = m_ActionDescription)

m_ActionNode = mapper_registry.map_imperatively(pycram.plan.ActionNode, t_ActionNode, polymorphic_identity = "ActionNode", inherits = m_DesignatorNode)

m_RepeatNode = mapper_registry.map_imperatively(pycram.language.RepeatNode, t_RepeatNode, polymorphic_identity = "RepeatNode", inherits = m_SequentialNode)

m_TransformStamped = mapper_registry.map_imperatively(pycram.datastructures.pose.TransformStamped, t_TransformStamped, properties = dict(pose=relationship('Transform',foreign_keys=[t_TransformStamped.c.pose_id])), polymorphic_identity = "TransformStamped", inherits = m_PoseStamped)

m_GraspPose = mapper_registry.map_imperatively(pycram.datastructures.pose.GraspPose, t_GraspPose, properties = dict(grasp_description=relationship('GraspDescription',foreign_keys=[t_GraspPose.c.grasp_description_id])), polymorphic_identity = "GraspPose", inherits = m_PoseStamped)

m_Transform = mapper_registry.map_imperatively(pycram.datastructures.pose.Transform, t_Transform, polymorphic_identity = "Transform", inherits = m_Pose)

m_Vector3Stamped = mapper_registry.map_imperatively(pycram.datastructures.pose.Vector3Stamped, t_Vector3Stamped, properties = dict(header=relationship('Header',foreign_keys=[t_Vector3Stamped.c.header_id])), polymorphic_identity = "Vector3Stamped", inherits = m_Vector3)
