from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from pycram.world_concepts.world_object import Object


class ProspectionObjectNotFound(KeyError):
    def __init__(self, obj: 'Object'):
        super().__init__(f"The given object {obj.name} is not in the prospection world.")


class WorldObjectNotFound(KeyError):
    def __init__(self, obj: 'Object'):
        super().__init__(f"The given object {obj.name} is not in the main world.")


class ObjectAlreadyExists(Exception):
    def __init__(self, obj: 'Object'):
        super().__init__(f"An object with the name {obj.name} already exists in the world.")


class ObjectDescriptionNotFound(KeyError):
    def __init__(self, object_name: str, path: str, extension: str):
        super().__init__(f"{object_name} with path {path} and extension {extension} is not in supported extensions, and"
                         f" the description data was not found on the ROS parameter server")
