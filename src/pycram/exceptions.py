from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from pycram.world_concepts.world_object import Object


class ProspectionObjectNotFound(KeyError):
    def __init__(self, obj: 'Object'):
        super().__init__(f"The given object {obj.name} is not in the prospection world.")


class WorldObjectNotFound(KeyError):
    def __init__(self, obj: 'Object'):
        super().__init__(f"The given object {obj.name} is not in the main world.")
