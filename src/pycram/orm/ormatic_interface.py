from pycram.orm.casts import StringType

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


t_ActionNodeDAO = Table(
    'ActionNodeDAO', metadata,
    Column('id', Integer, primary_key=True)
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
    Column('frame_id', String, nullable=False),
    Column('stamp', DateTime, nullable=False),
    Column('sequence', Integer, nullable=False)
)

t_MotionNodeDAO = Table(
    'MotionNodeDAO', metadata,
    Column('id', Integer, primary_key=True)
)

t_PlanNode = Table(
    'PlanNode', metadata,
    Column('id', Integer, primary_key=True),
    Column('status', Enum(pycram.datastructures.enums.TaskStatus), nullable=False),
    Column('start_time', DateTime),
    Column('end_time', DateTime),
    Column('polymorphic_type', String)
)

t_Quaternion = Table(
    'Quaternion', metadata,
    Column('id', Integer, primary_key=True),
    Column('x', Float, nullable=False),
    Column('y', Float, nullable=False),
    Column('z', Float, nullable=False),
    Column('w', Float, nullable=False)
)

t_RepeatNode = Table(
    'RepeatNode', metadata,
    Column('id', Integer, primary_key=True),
    Column('status', Enum(pycram.datastructures.enums.TaskStatus), nullable=False),
    Column('start_time', DateTime),
    Column('end_time', DateTime),
    Column('action', TypeType),
    Column('repeat', Integer, nullable=False)
)

t_Vector3 = Table(
    'Vector3', metadata,
    Column('id', Integer, primary_key=True),
    Column('x', Float, nullable=False),
    Column('y', Float, nullable=False),
    Column('z', Float, nullable=False)
)

t_LanguageNode = Table(
    'LanguageNode', metadata,
    Column('id', ForeignKey('PlanNode.id'), primary_key=True),
    Column('action', TypeType)
)

t_Pose = Table(
    'Pose', metadata,
    Column('id', Integer, primary_key=True),
    Column('position_id', ForeignKey('Vector3.id'), nullable=False),
    Column('orientation_id', ForeignKey('Quaternion.id'), nullable=False),
    Column('polymorphic_type', String)
)

t_PoseStamped = Table(
    'PoseStamped', metadata,
    Column('id', Integer, primary_key=True),
    Column('pose_id', ForeignKey('Pose.id'), nullable=False),
    Column('header_id', ForeignKey('Header.id'), nullable=False),
    Column('polymorphic_type', String)
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
    Column('polymorphic_type', String)
)

t_FrozenObjectDAO = Table(
    'FrozenObjectDAO', metadata,
    Column('id', Integer, primary_key=True),
    Column('name', String, nullable=False),
    Column('concept', StringType, nullable=False),
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
    Column('child_frame_id', String, nullable=False)
)

t_CloseAction = Table(
    'CloseAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('grasping_prepose_distance', Float, nullable=False)
)

t_DetectActionDAO = Table(
    'DetectActionDAO', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('technique', Enum(pycram.datastructures.enums.DetectionTechnique), nullable=False),
    Column('state', Enum(pycram.datastructures.enums.DetectionState)),
    Column('region', String)
)

t_FaceAtAction = Table(
    'FaceAtAction', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('pose_id', ForeignKey('PoseStamped.id'), nullable=False),
    Column('keep_joint_states', Boolean, nullable=False)
)

t_GraspingActionDAO = Table(
    'GraspingActionDAO', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False)
)

t_GripActionDAO = Table(
    'GripActionDAO', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
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

t_PickUpActionDAO = Table(
    'PickUpActionDAO', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False)
)

t_PlaceActionDAO = Table(
    'PlaceActionDAO', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('target_location_id', ForeignKey('PoseStamped.id'), nullable=False)
)

t_ReachToPickUpActionDAO = Table(
    'ReachToPickUpActionDAO', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('grasp_description_id', ForeignKey('GraspDescription.id'), nullable=False)
)

t_ReleaseActionDAO = Table(
    'ReleaseActionDAO', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('gripper', Enum(pycram.datastructures.enums.Arms), nullable=False)
)

t_ResolvedActionNodeDAO = Table(
    'ResolvedActionNodeDAO', metadata,
    Column('id', Integer, primary_key=True),
    Column('designator_ref_id', ForeignKey('ActionDescription.id'), nullable=False)
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

t_TransportActionDAO = Table(
    'TransportActionDAO', metadata,
    Column('id', ForeignKey('ActionDescription.id'), primary_key=True),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False),
    Column('target_location_id', ForeignKey('PoseStamped.id'), nullable=False)
)

mapper_registry = registry(metadata=metadata)

m_PoseStamped = mapper_registry.map_imperatively(pycram.datastructures.pose.PoseStamped, t_PoseStamped, properties = dict(pose=relationship('Pose',foreign_keys=[t_PoseStamped.c.pose_id]), 
header=relationship('Header',foreign_keys=[t_PoseStamped.c.header_id])), polymorphic_on = "polymorphic_type", polymorphic_identity = "PoseStamped")

m_Header = mapper_registry.map_imperatively(pycram.datastructures.pose.Header, t_Header, )

m_Vector3 = mapper_registry.map_imperatively(pycram.datastructures.pose.Vector3, t_Vector3, )

m_Pose = mapper_registry.map_imperatively(pycram.datastructures.pose.Pose, t_Pose, properties = dict(position=relationship('Vector3',foreign_keys=[t_Pose.c.position_id]), 
orientation=relationship('Quaternion',foreign_keys=[t_Pose.c.orientation_id])), polymorphic_on = "polymorphic_type", polymorphic_identity = "Pose")

m_Quaternion = mapper_registry.map_imperatively(pycram.datastructures.pose.Quaternion, t_Quaternion, )

m_GraspDescription = mapper_registry.map_imperatively(pycram.datastructures.grasp.GraspDescription, t_GraspDescription, )

m_ActionDescription = mapper_registry.map_imperatively(pycram.designator.ActionDescription, t_ActionDescription, properties = dict(robot_position=relationship('PoseStamped',foreign_keys=[t_ActionDescription.c.robot_position_id]), 
robot_type=t_ActionDescription.c.robot_type), polymorphic_on = "polymorphic_type", polymorphic_identity = "ActionDescription")

m_PlanNode = mapper_registry.map_imperatively(pycram.plan.PlanNode, t_PlanNode, polymorphic_on = "polymorphic_type", polymorphic_identity = "PlanNode")

m_RepeatNode = mapper_registry.map_imperatively(pycram.language.RepeatNode, t_RepeatNode, properties = dict(action=t_RepeatNode.c.action))

m_ActionNodeDAO = mapper_registry.map_imperatively(pycram.plan.ActionNode, t_ActionNodeDAO, )

m_MotionNodeDAO = mapper_registry.map_imperatively(pycram.plan.MotionNode, t_MotionNodeDAO, )

m_ResolvedActionNodeDAO = mapper_registry.map_imperatively(pycram.plan.ResolvedActionNode, t_ResolvedActionNodeDAO, properties = dict(designator_ref=relationship('ActionDescription',foreign_keys=[t_ResolvedActionNodeDAO.c.designator_ref_id])))

m_FrozenObjectDAO = mapper_registry.map_imperatively(pycram.datastructures.dataclasses.FrozenObject, t_FrozenObjectDAO, properties = dict(pose=relationship('PoseStamped',foreign_keys=[t_FrozenObjectDAO.c.pose_id]), 
concept=t_FrozenObjectDAO.c.concept))

m_GraspPose = mapper_registry.map_imperatively(pycram.datastructures.pose.GraspPose, t_GraspPose, properties = dict(grasp_description=relationship('GraspDescription',foreign_keys=[t_GraspPose.c.grasp_description_id])), polymorphic_identity = "GraspPose", inherits = m_PoseStamped)

m_TransformStamped = mapper_registry.map_imperatively(pycram.datastructures.pose.TransformStamped, t_TransformStamped, properties = dict(pose=relationship('Transform',foreign_keys=[t_TransformStamped.c.pose_id])), polymorphic_identity = "TransformStamped", inherits = m_PoseStamped)

m_Transform = mapper_registry.map_imperatively(pycram.datastructures.pose.Transform, t_Transform, polymorphic_identity = "Transform", inherits = m_Pose)

m_MoveTorsoAction = mapper_registry.map_imperatively(pycram.designators.action_designator.MoveTorsoAction, t_MoveTorsoAction, polymorphic_identity = "MoveTorsoAction", inherits = m_ActionDescription)

m_SetGripperAction = mapper_registry.map_imperatively(pycram.designators.action_designator.SetGripperAction, t_SetGripperAction, polymorphic_identity = "SetGripperAction", inherits = m_ActionDescription)

m_ParkArmsAction = mapper_registry.map_imperatively(pycram.designators.action_designator.ParkArmsAction, t_ParkArmsAction, polymorphic_identity = "ParkArmsAction", inherits = m_ActionDescription)

m_NavigateAction = mapper_registry.map_imperatively(pycram.designators.action_designator.NavigateAction, t_NavigateAction, properties = dict(target_location=relationship('PoseStamped',foreign_keys=[t_NavigateAction.c.target_location_id])), polymorphic_identity = "NavigateAction", inherits = m_ActionDescription)

m_LookAtAction = mapper_registry.map_imperatively(pycram.designators.action_designator.LookAtAction, t_LookAtAction, properties = dict(target=relationship('PoseStamped',foreign_keys=[t_LookAtAction.c.target_id])), polymorphic_identity = "LookAtAction", inherits = m_ActionDescription)

m_OpenAction = mapper_registry.map_imperatively(pycram.designators.action_designator.OpenAction, t_OpenAction, polymorphic_identity = "OpenAction", inherits = m_ActionDescription)

m_CloseAction = mapper_registry.map_imperatively(pycram.designators.action_designator.CloseAction, t_CloseAction, polymorphic_identity = "CloseAction", inherits = m_ActionDescription)

m_FaceAtAction = mapper_registry.map_imperatively(pycram.designators.action_designator.FaceAtAction, t_FaceAtAction, properties = dict(pose=relationship('PoseStamped',foreign_keys=[t_FaceAtAction.c.pose_id])), polymorphic_identity = "FaceAtAction", inherits = m_ActionDescription)

m_SearchAction = mapper_registry.map_imperatively(pycram.designators.action_designator.SearchAction, t_SearchAction, properties = dict(target_location=relationship('PoseStamped',foreign_keys=[t_SearchAction.c.target_location_id]), 
object_type=t_SearchAction.c.object_type), polymorphic_identity = "SearchAction", inherits = m_ActionDescription)

m_MoveAndPickUpAction = mapper_registry.map_imperatively(pycram.designators.action_designator.MoveAndPickUpAction, t_MoveAndPickUpAction, properties = dict(standing_position=relationship('PoseStamped',foreign_keys=[t_MoveAndPickUpAction.c.standing_position_id]), 
grasp_description=relationship('GraspDescription',foreign_keys=[t_MoveAndPickUpAction.c.grasp_description_id])), polymorphic_identity = "MoveAndPickUpAction", inherits = m_ActionDescription)

m_ReleaseActionDAO = mapper_registry.map_imperatively(pycram.designators.action_designator.ReleaseAction, t_ReleaseActionDAO, polymorphic_identity = "ReleaseActionDAO", inherits = m_ActionDescription)

m_GripActionDAO = mapper_registry.map_imperatively(pycram.designators.action_designator.GripAction, t_GripActionDAO, polymorphic_identity = "GripActionDAO", inherits = m_ActionDescription)

m_ReachToPickUpActionDAO = mapper_registry.map_imperatively(pycram.designators.action_designator.ReachToPickUpAction, t_ReachToPickUpActionDAO, properties = dict(grasp_description=relationship('GraspDescription',foreign_keys=[t_ReachToPickUpActionDAO.c.grasp_description_id])), polymorphic_identity = "ReachToPickUpActionDAO", inherits = m_ActionDescription)

m_PickUpActionDAO = mapper_registry.map_imperatively(pycram.designators.action_designator.PickUpAction, t_PickUpActionDAO, polymorphic_identity = "PickUpActionDAO", inherits = m_ActionDescription)

m_PlaceActionDAO = mapper_registry.map_imperatively(pycram.designators.action_designator.PlaceAction, t_PlaceActionDAO, properties = dict(target_location=relationship('PoseStamped',foreign_keys=[t_PlaceActionDAO.c.target_location_id])), polymorphic_identity = "PlaceActionDAO", inherits = m_ActionDescription)

m_TransportActionDAO = mapper_registry.map_imperatively(pycram.designators.action_designator.TransportAction, t_TransportActionDAO, properties = dict(target_location=relationship('PoseStamped',foreign_keys=[t_TransportActionDAO.c.target_location_id])), polymorphic_identity = "TransportActionDAO", inherits = m_ActionDescription)

m_DetectActionDAO = mapper_registry.map_imperatively(pycram.designators.action_designator.DetectAction, t_DetectActionDAO, polymorphic_identity = "DetectActionDAO", inherits = m_ActionDescription)

m_GraspingActionDAO = mapper_registry.map_imperatively(pycram.designators.action_designator.GraspingAction, t_GraspingActionDAO, polymorphic_identity = "GraspingActionDAO", inherits = m_ActionDescription)

m_LanguageNode = mapper_registry.map_imperatively(pycram.language.LanguageNode, t_LanguageNode, properties = dict(action=t_LanguageNode.c.action), polymorphic_identity = "LanguageNode", inherits = m_PlanNode)
