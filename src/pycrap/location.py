from .base import Base

class Location(Base):
    """
    A physical region.
    """

class Room(Location):
    """
    A region in a building.
    """

class Bedroom(Room):
    """
    A room where people sleep in.
    """

class Kitchen(Room):
    """
    A room where food is prepared.
    """

class LivingRoom(Room):
    """
    A room where people relax in.
    """

class Bathroom(Room):
    """
    A room where people wash in.
    """

class Apartment(Location):
    """
    A building with multiple rooms.
    """