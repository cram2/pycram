from .dependencies import *
from .classes import *


class has_color_value(BaseProperty):
    """
    Associates a ColorRegion to numerical data describing the color.
    """
    
class has_region_data_value(BaseProperty):
    """
    A datatype property that encodes values for a Region, e.g. a float for the Region Height.
    """
    
class has_data_format(BaseProperty):
    """
    A property linking an InformationRealization to a string specifying a format name, e.g. URDF or STL.
    """
    
class has_data_value(BaseProperty):
    """
    A datatype property that encodes values from a datatype for an Entity. 
    There are several ways to encode values in DOLCE (Ultralite):
    
    1) Directly assert an xsd:_ value to an Entity by using hasDataValue
    2) Assert a Region for an Entity by using hasRegion, and then assert an xsd:_ value to that Region, by using hasRegionDataValue
    3) Assert a Quality for an Entity by using hasQuality, then assert a Region for that Quality, and assert an xsd:_ value to that Region, by using hasRegionDataValue
    4) When the value is required, but not directly observed, assert a Parameter for an xsd:_ value by using hasParameterDataValue, and then associate the Parameter to an Entity by using isConstraintFor
    5) When the value is required, but not directly observed, you can also assert a Parameter for a Region by using parametrizes, and then assert an xsd:_ value to that Region, by using hasRegionDataValue
    
    The five approaches obey different requirements. 
    For example, a simple value can be easily asserted by using pattern (1), but if one needs to assert an interval between two values, a Region should be introduced to materialize that interval, as pattern (2) suggests. 
    Furthermore, if one needs to distinguish the individual Quality of a value, e.g. the particular nature of the density of a substance, pattern (3) can be used. 
    Patterns (4) and (5) should be used instead when a constraint or a selection is modeled, independently from the actual observation of values in the real world.
    """
    
class has_depth(BaseProperty):
    """
    The depth of a shape.
    """
    
class has_shape_parameter(BaseProperty):
    """
    Associates a SpaceRegion to some parameter value describing its shape. This is a fairly generic property, and to capture the semantics of the information associated to the SpaceRegion, its more specific subproperties should be used.
    """
    
class has_event_begin(BaseProperty):
    """
    A relation recording when an Event started. In this case, we think of the Event as something unfolding over some span of time.
    """
    
class has_event_time(BaseProperty):
    """
    Superproperty of hasEventBegin and hasEventEnd, records that an Event happened, or was happening, at a particular time. Using the subproperties captures the richer semantics of that time relative to the event. Using only this superproperty may be appropriate when the Event is construed to take place at a single instant of time.
    """
    
class has_event_end(BaseProperty):
    """
    A relation recording when an Event ended. In this case, we think of the Event as something unfolding over some span of time.
    """
    
class has_file_path(BaseProperty):
    """
    Associates an entity to some file containing data about it. For example, can be used to describe physical objects via a mesh file.
    """
    
class has_force_value(BaseProperty):
    """
    A value that quantifies a force given in Newton.
    """
    
class has_friction_value(BaseProperty):
    """
    The coefficient of friction denotes the ratio of friction force between touching objects. The coefficient is dimensionless.
    """
    
class has_hsv_value(BaseProperty):
    """
    Associates a ColorRegion to numerical data describing the color. This data uses the Hue-Saturation-Value color space.
    """
    
class has_height(BaseProperty):
    """
    The height of a shape.
    """
    
class has_interval_begin(BaseProperty):
    """
    A relation recording when some TimeInterval started.
    """
    
class has_interval_time(BaseProperty):
    """
    Superproperty of relations used to connect moments in time to a TimeInterval.
    """
    
class has_interval_end(BaseProperty):
    """
    A relation recording when a TimeInterval ended.
    """
    
class has_joint_effort(BaseProperty):
    """
    The effort applied in a joint given in N (prismatic joint) or N*m (hinged joints).
    """
    
class has_joint_parameter(BaseProperty):
    """
    Assigns a value for an attribute of a joint.
    """
    
class has_joint_effort_limit(BaseProperty):
    """
    The maximum effort applied in a joint given in N (prismatic joint) or N*m (hinged joints).
    """
    
class has_joint_position(BaseProperty):
    """
    The position of a joint given in m (prismatic joints) or rad (hinged joints).
    """
    
class has_joint_position_max(BaseProperty):
    """
    The maximum position of a joint given in m (prismatic joints) or rad (hinged joints).
    """
    
class has_joint_position_min(BaseProperty):
    """
    The minimum position of a joint given in m (prismatic joints) or rad (hinged joints).
    """
    
class has_joint_velocity(BaseProperty):
    """
    The velocity of a joint given in m/s (prismatic joints) or rad/s (hinged joints).
    """
    
class has_joint_velocity_limit(BaseProperty):
    """
    The maximum velocity of a joint given in m/s (prismatic joints) or rad/s (hinged joints).
    """
    
class has_length(BaseProperty):
    """
    The length of a shape.
    """
    
class has_mass_value(BaseProperty):
    """
    The mass value of a physical object in kilogram.
    """
    
class has_name_string(BaseProperty):
    """
    A relation recording some identifier associated to an Entity.
    """
    
class has_persistent_identifier(BaseProperty):
    """
    A property linking an InformationRealization to a persistent identifier such as a DOI, which can then be used to obtain an address at which the realization (i.e. digital file) can be retrieved.
    """
    
class has_position_data(BaseProperty):
    """
    Associates a spatial region to a position.
    """
    
class has_space_parameter(BaseProperty):
    """
    Associates a SpaceRegion to some parameter value describing it. This is a fairly generic property, and to capture the semantics of the information associated to the SpaceRegion, its more specific subproperties should be used.
    """
    
class has_priority(BaseProperty):
    """
    A relation asserting some entity has a particular priority.
    """
    
class has_rgb_value(BaseProperty):
    """
    Associates a ColorRegion to numerical data describing the color. This data uses the Red-Green-Blue color space.
    """
    
class has_radius(BaseProperty):
    """
    The radius of a circular or oval shape.
    """
    
class has_reference_frame(BaseProperty):
    """
    Gives the name associated to the local coordinate frame of a SpaceRegion.
    """
    
class has_shape_scale(BaseProperty):
    """
    The scale of a shape, given as a vector of three real numbers to adjust x, y, z components of vertex vectors. In cases where a shape needs to be flipped compared to the shape described by a mesh, one of the scale components will be negative. 
    
    It is often the case that shapes need to be altered from their description in a shape file, and a typical example of this is scaling a mesh. 
    
    In robotics, it is not uncommon to encounter shapes that are flipped compared to the shape in a mesh file. This is because robots often have bilateral symmetry, thus it makes sense to reuse the same meshes for corresponding links of the left and right arms.
    """
    
class has_width(BaseProperty):
    """
    The width of a shape.
    """
    
class is_reification_of(BaseProperty):
    """
    An auxiliary property that is used to generate object individuals, called reifications, from any other Entity, e.g. from relations, classes, data types. These reifications can then be used in DL axioms as any other named individual.
    """
    
