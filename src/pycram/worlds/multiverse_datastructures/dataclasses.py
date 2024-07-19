from dataclasses import dataclass

from typing_extensions import List


@dataclass
class RayResult:
    """
    A dataclass to store the ray result. The ray result contains the body name that the ray intersects with and the
    distance from the ray origin to the intersection point.
    """
    body_name: str
    distance: float

    def intersected(self) -> bool:
        """
        Check if the ray intersects with a body.
        return: Whether the ray intersects with a body.
        """
        return self.distance >= 0 and self.body_name != ""


@dataclass
class MultiverseContactPoint:
    """
    A dataclass to store the contact point returned from Multiverse.
    """
    body_name: str
    contact_force: List[float]
    contact_torque: List[float]
