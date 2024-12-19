from .dependencies import *
from .classes import *
from .individuals import *


Entity.is_a = [Thing]

DesignedArtifact.is_a = [PhysicalArtifact, is_described_by.some(Design)]

PhysicalObject.is_a = [Object, has_part.only(PhysicalObject)]

Quality.is_a = [Entity, has_region.some(Region), is_quality_of.some(Entity), has_constituent.only(Quality), has_part.only(Quality)]

Region.is_a = [Abstract, has_constituent.only(Region), has_part.only(Region), overlaps.only(Region), precedes.only(Region)]

FrictionAttribute.is_a = [PhysicalAttribute, has_friction_value.exactly(1, float)]

MassAttribute.is_a = [PhysicalAttribute, has_mass_value.exactly(1, float)]

Shape.is_a = [Intrinsic, has_region.some(ShapeRegion), has_region.only(ShapeRegion)]

ShapeRegion.is_a = [Region, has_space_region.max(1, SixDPose)]

SpaceRegion.is_a = [Region]

PhysicalAttribute.is_a = [Region, is_region_for.only(PhysicalObject)]

PhysicalAgent.is_a = [Agent, PhysicalObject]

BoxShape.is_a = [ShapeRegion, has_height.exactly(1, float), has_length.exactly(1, float), has_width.exactly(1, float)]

CylinderShape.is_a = [ShapeRegion, has_radius.some(float), has_length.exactly(1, float)]

Configuration.is_a = [Collection]

MeshShape.is_a = [ShapeRegion, has_file_path.exactly(1, str), has_shape_scale.max(1, float)]

SixDPose.is_a = [SpaceRegion, has_position_data.exactly(1, str)]

SphereShape.is_a = [ShapeRegion, has_radius.exactly(1, float)]

Link.is_a = [PhysicalObject]

Joint.is_a = [PhysicalObject, has_child_link.some(PhysicalObject), has_parent_link.some(PhysicalObject), has_urdf_name.some(str)]
Joint.equivalent_to = [Or([FixedJoint, MovableJoint])]

DampingAttribute.is_a = [PhysicalAttribute, has_damping_value.exactly(1, float)]

Inertia.is_a = [PhysicalAttribute, has_inertia_ixx.exactly(1, float), has_inertia_ixy.exactly(1, float), has_inertia_ixz.exactly(1, float), has_inertia_iyy.exactly(1, float), has_inertia_iyz.exactly(1, float), has_inertia_izz.exactly(1, float)]

JointAxis.is_a = [JointAttribute]

JointLimits.is_a = [JointAttribute, has_lower_limit.exactly(1, float), has_max_joint_effort.exactly(1, float), has_max_joint_velocity.exactly(1, float), has_upper_limit.exactly(1, float)]

JointPosition.is_a = [SpaceRegion]

JointReferencePositions.is_a = [JointAttribute, has_falling_edge.only(float), has_rising_edge.only(float)]

JointSoftLimits.is_a = [JointAttribute, has_k_velocity.some(float), has_k_position.only(float), has_lower_limit.only(float), has_upper_limit.only(float)]

ContinuousJoint.is_a = [HingeJoint]

HingeJoint.is_a = [MovableJoint, has_joint_axis.exactly(1, JointAxis)]

FixedJoint.is_a = [Joint]

MovableJoint.is_a = [Joint]

FloatingJoint.is_a = [MovableJoint]

JointAttribute.is_a = [PhysicalAttribute]

PlanarJoint.is_a = [MovableJoint, has_joint_axis.exactly(1, JointAxis)]

PrismaticJoint.is_a = [MovableJoint, has_joint_axis.exactly(1, JointAxis), has_joint_limits.exactly(1, JointLimits)]

RevoluteJoint.is_a = [HingeJoint, has_joint_limits.exactly(1, JointLimits)]

has_name_string.is_a = [DatatypeProperty, has_data_value]
has_name_string.domain = [Entity]
has_name_string.range = [str]

has_data_value.is_a = [DatatypeProperty]
has_data_value.domain = [Entity]

has_region_data_value.is_a = [DatatypeProperty, has_data_value]
has_region_data_value.domain = [Region]

has_axis_vector.is_a = [DatatypeProperty, has_region_data_value]
has_axis_vector.domain = [JointAxis]
has_axis_vector.range = [float]

has_base_link_name.is_a = [DatatypeProperty, has_name_string]
has_base_link_name.domain = [PhysicalObject]
has_base_link_name.range = [str]

has_damping_value.is_a = [DatatypeProperty, has_region_data_value]
has_damping_value.domain = [DampingAttribute]
has_damping_value.range = [float]

has_end_link_name.is_a = [DatatypeProperty, has_name_string]
has_end_link_name.domain = [PhysicalObject]
has_end_link_name.range = [str]

has_falling_edge.is_a = [DatatypeProperty, has_data_value]
has_falling_edge.domain = [JointReferencePositions]
has_falling_edge.range = [float]

has_inertia_ixx.is_a = [DatatypeProperty, has_region_data_value]
has_inertia_ixx.domain = [Inertia]
has_inertia_ixx.range = [float]

has_inertia_ixy.is_a = [DatatypeProperty, has_region_data_value]
has_inertia_ixy.domain = [Inertia]
has_inertia_ixy.range = [float]

has_inertia_ixz.is_a = [DatatypeProperty, has_region_data_value]
has_inertia_ixz.domain = [Inertia]
has_inertia_ixz.range = [float]

has_inertia_iyy.is_a = [DatatypeProperty, has_region_data_value]
has_inertia_iyy.domain = [Inertia]
has_inertia_iyy.range = [float]

has_inertia_iyz.is_a = [DatatypeProperty, has_region_data_value]
has_inertia_iyz.domain = [Inertia]
has_inertia_iyz.range = [float]

has_inertia_izz.is_a = [DatatypeProperty, has_region_data_value]
has_inertia_izz.domain = [Inertia]
has_inertia_izz.range = [float]

has_k_position.is_a = [DatatypeProperty, has_data_value]
has_k_position.domain = [JointSoftLimits]
has_k_position.range = [float]

has_k_velocity.is_a = [DatatypeProperty, has_data_value]
has_k_velocity.domain = [JointSoftLimits]
has_k_velocity.range = [float]

has_lower_limit.is_a = [DatatypeProperty, has_data_value]
has_lower_limit.domain = [Entity]
has_lower_limit.range = [float]

has_max_joint_effort.is_a = [DatatypeProperty, has_data_value]
has_max_joint_effort.domain = [JointLimits]
has_max_joint_effort.range = [float]

has_max_joint_velocity.is_a = [DatatypeProperty, has_data_value]
has_max_joint_velocity.domain = [JointLimits]
has_max_joint_velocity.range = [float]

has_rising_edge.is_a = [DatatypeProperty, has_data_value]
has_rising_edge.domain = [JointReferencePositions]
has_rising_edge.range = [float]

has_urdf_name.is_a = [DatatypeProperty, has_name_string]
has_urdf_name.domain = [PhysicalObject]
has_urdf_name.range = [str]

has_upper_limit.is_a = [DatatypeProperty, has_data_value]
has_upper_limit.domain = [JointReferencePositions]
has_upper_limit.range = [float]

has_physical_component.is_a = [ObjectProperty, has_component]
has_physical_component.domain = [PhysicalObject]
has_physical_component.range = [PhysicalObject]

has_constituent.is_a = [ObjectProperty, associated_with]
has_constituent.domain = [Entity]
has_constituent.range = [Entity]

has_region.is_a = [ObjectProperty, associated_with]
has_region.domain = [Entity]
has_region.range = [Region]

has_quality.is_a = [ObjectProperty, associated_with]
has_quality.domain = [Entity]
has_quality.range = [Quality]

is_region_for.is_a = [ObjectProperty, associated_with]
is_region_for.domain = [Region]
is_region_for.range = [Entity]

is_quality_of.is_a = [ObjectProperty, associated_with]
is_quality_of.domain = [Quality]
is_quality_of.range = [Entity]

has_base_link.is_a = [ObjectProperty, has_root_link]
has_base_link.domain = [PhysicalObject]
has_base_link.range = [Link]

has_root_link.is_a = [ObjectProperty, has_link]
has_root_link.domain = [PhysicalObject]
has_root_link.range = [PhysicalObject]

has_child_link.is_a = [ObjectProperty, has_constituent]
has_child_link.domain = [Joint]
has_child_link.range = [PhysicalObject]

has_collision_shape.is_a = [ObjectProperty, has_quality]
has_collision_shape.domain = [PhysicalObject]
has_collision_shape.range = [Shape]

has_damping_attribute.is_a = [ObjectProperty, has_region]
has_damping_attribute.domain = [PhysicalObject]
has_damping_attribute.range = [DampingAttribute]

has_end_link.is_a = [ObjectProperty, has_link]
has_end_link.domain = [PhysicalObject]
has_end_link.range = [Link]

has_link.is_a = [ObjectProperty, has_physical_component]
has_link.domain = [PhysicalObject]
has_link.range = [PhysicalObject]

has_inertia.is_a = [ObjectProperty, has_region]
has_inertia.domain = [PhysicalObject]
has_inertia.range = [Inertia]

has_joint.is_a = [ObjectProperty, has_physical_component]
has_joint.domain = [PhysicalObject]
has_joint.range = [Joint]

has_joint_axis.is_a = [ObjectProperty, has_region]
has_joint_axis.domain = [Joint]
has_joint_axis.range = [JointAxis]

has_joint_limits.is_a = [ObjectProperty, has_region]
has_joint_limits.domain = [Joint]
has_joint_limits.range = [JointLimits]

has_joint_position.is_a = [ObjectProperty, has_region]
has_joint_position.domain = [Joint]
has_joint_position.range = [JointPosition]

has_joint_reference_positions.is_a = [ObjectProperty, has_region]
has_joint_reference_positions.domain = [Joint]
has_joint_reference_positions.range = [JointReferencePositions]

has_joint_soft_limits.is_a = [ObjectProperty, has_region]
has_joint_soft_limits.domain = [Joint]
has_joint_soft_limits.range = [JointSoftLimits]

has_origin.is_a = [ObjectProperty, has_region]
has_origin.domain = [Entity]
has_origin.range = [SixDPose]

has_parent_link.is_a = [ObjectProperty, has_constituent]
has_parent_link.domain = [Joint]
has_parent_link.range = [PhysicalObject]

