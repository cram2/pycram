from .base import Base


class Link(Base):
    """
    A link is a basic element of the URDF.
    """


class Joint(Base):
    """
    A joint is an element of a URDF that connects two links.
    """


class FixedJoint(Joint):
    """
    A fixed joint is a joint that does not allow any movement.
    """