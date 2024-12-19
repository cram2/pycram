from .dependencies import *


class has_disposition(BaseProperty):
    """
    Associates an object to one of its dispositions.
    Associates an object to one of its dispositions.
    """
    
class has_physical_component(BaseProperty):
    """
    A relation used to describe the structure of a PhysicalObject in terms of physical components, i.e. what other PhysicalObjects are components of it.
    """
    
class has_destination_location(BaseProperty):
    ...
    
class has_predefined_location(BaseProperty):
    ...
    
class has_origin_location(BaseProperty):
    ...
    
class associated_with(BaseProperty):
    """
    A catch-all object property, useful for alignment and querying purposes.
    It is declared as both transitive and symmetric, in order to reason an a maximal closure of associations between individuals.
    """
    
class is_entry_to(BaseProperty):
    """
    A spatial relation holding between a room and a location on the border of the room that should be used for entering the room.
    """
    
class has_location(BaseProperty):
    """
    A generic, relative spatial location, holding between any entities. E.g. 'the cat is on the mat', 'Omar is in Samarcanda', 'the wound is close to the femural artery'.
    For 'absolute' locations, see SpaceRegion
    """
    
class is_exit_from(BaseProperty):
    """
    A spatial relation holding between a room and a location on the border of the room that should be used for exiting the room.
    """
    
class is_in_room(BaseProperty):
    ...
    
