from .dependencies import *
from .classes import *


class has_name_string(BaseProperty):
    """
    A relation recording some identifier associated to an Entity.
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
    
class has_region_data_value(BaseProperty):
    """
    A datatype property that encodes values for a Region, e.g. a float for the Region Height.
    """
    
class has_axis_vector(BaseProperty):
    ...
    
class has_base_link_name(BaseProperty):
    ...
    
class has_damping_value(BaseProperty):
    ...
    
class has_end_link_name(BaseProperty):
    ...
    
class has_falling_edge(BaseProperty):
    ...
    
class has_inertia_ixx(BaseProperty):
    ...
    
class has_inertia_ixy(BaseProperty):
    ...
    
class has_inertia_ixz(BaseProperty):
    ...
    
class has_inertia_iyy(BaseProperty):
    ...
    
class has_inertia_iyz(BaseProperty):
    ...
    
class has_inertia_izz(BaseProperty):
    ...
    
class has_k_position(BaseProperty):
    ...
    
class has_k_velocity(BaseProperty):
    ...
    
class has_lower_limit(BaseProperty):
    ...
    
class has_max_joint_effort(BaseProperty):
    ...
    
class has_max_joint_velocity(BaseProperty):
    ...
    
class has_rising_edge(BaseProperty):
    ...
    
class has_urdf_name(BaseProperty):
    """
    The name of the link or joint.
    """
    
class has_upper_limit(BaseProperty):
    ...
    
