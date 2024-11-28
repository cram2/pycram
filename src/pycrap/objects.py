from . import default_pycrap_ontology
from .base import PhysicalObject, Base, BaseProperty


class Container(PhysicalObject):
    """
    Any object that can contain other objects.
    """


class Cup(Container):
    """
    A cup is a small open container used for drinking.
    """


class Mug(Container):
    equivalent_to = [Cup]


class MetalMug(Mug):
    """
    A mug made of metal.
    """


class Food(PhysicalObject):
    """
    Any substance that can be consumed by living organisms.
    """


class Pringles(Food):
    """
    A brand of potato snack chips.
    """


class Milk(Food):
    """
    A white liquid produced by the mammary glands of mammals.
    """


class Cutlery(PhysicalObject):
    """
    Any implement, tool, or container used for serving or eating food.
    """


class Fork(Cutlery):
    """
    A fork is a tool consisting of a handle with several narrow tines on one end.
    """


class Spoon(Cutlery):
    """
    A spoon is a utensil consisting of a small shallow bowl oval or round in shape, with a handle.
    """


class Knife(Cutlery):
    """
    A knife is a tool with a cutting edge or blade attached to a handle.
    """


class Plate(PhysicalObject):
    """
    A plate is a broad, concave, but mainly flat vessel on which food can be served.
    """


class Bowl(Container, Plate):
    """
    A bowl is a round, open-top container used in many cultures to serve food.
    """


class Cereal(Food):
    """
    A traditional breakfast dish made from processed cereal grains.
    """


class Floor(PhysicalObject):
    """
    The lower surface of a room.
    """


class Table(PhysicalObject):
    """
    A table is an item of furniture with a flat top, used as a surface for working at,
    eating from or on which to place things.
    """


class Board(PhysicalObject):
    """
    A flat, thin, rectangular supporting piece.
    """


class has_part(BaseProperty, PhysicalObject >> PhysicalObject):
    """
    A property that relates an object to a part of it.
    """