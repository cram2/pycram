from .dependencies import *


class Entity(Base):
    """
    Anything: real, possible, or imaginary, which some modeller wants to talk about for some purpose.
    """
    

class DesignedArtifact(Base):
    """
    A PhysicalArtifact that is also described by a Design. This excludes simple recycling or refunctionalization of natural objects. Most common sense 'artifacts' can be included in this class: cars, lamps, houses, chips, etc.
    """
    

class PhysicalObject(Base):
    """
    Any Object that has a proper space region. The prototypical physical object has also an associated mass, but the nature of its mass can greatly vary based on the epistemological status of the object (scientifically measured, subjectively possible, imaginary).
    """
    

class Quality(Base):
    """
    Any aspect of an Entity (but not a part of it), which cannot exist without that Entity. For example, the way the surface of a specific PhysicalObject looks like, or the specific light of a place at a certain time, are examples of Quality, while the encoding of a Quality into e.g. a PhysicalAttribute should be modeled as a Region. 
    From the design viewpoint, the Quality-Region distinction is useful only when individual aspects of an Entity are considered in a domain of discourse. 
    For example, in an automotive context, it would be irrelevant to consider the aspects of car windows for a specific car, unless the factory wants to check a specific window against design parameters (anomaly detection). 
    On the other hand, in an antiques context, the individual aspects for a specific piece of furniture are a major focus of attention, and may constitute the actual added value, because the design parameters for old furniture are often not fixed, and may not be viewed as 'anomalies'.
    """
    

class Region(Base):
    """
    Any region in a dimensional space (a dimensional space is a maximal Region), which can be used as a value for a quality of an Entity . For example, TimeInterval, SpaceRegion, PhysicalAttribute, Amount, SocialAttribute are all subclasses of Region. 
    Regions are not data values in the ordinary knowledge representation sense; in order to get patterns for modelling data, see the properties: representsDataValue and hasDataValue
    """
    

class FrictionAttribute(Base):
    """
    The resistance that one surface or object encounters when moving over another.
    """
    

class MassAttribute(Base):
    """
    The quantity of matter which a body contains, as measured by its acceleration under given force or by the force exerted on it by a gravitational field.
    """
    

class Shape(Base):
    """
    The external form, contours, or outline of an object.
    """
    

class ShapeRegion(Base):
    """
    Encodes the shape of an object.
    
    Note that sometimes the shape as actually used for some purpose may be displaced. This is the case, e.g., for robot links which use a mesh file to describe their shape, but the reference pose of the link uses the mesh translated/rotated in the link's local coordinate frame.
    """
    

class SpaceRegion(Base):
    """
    Any Region in a dimensional space that is used to localize an Entity ; i.e., it is not used to represent some characteristic (e.g. it excludes time intervals, colors, size values, judgment values, etc.). Differently from a Place , a space region has a specific dimensional space.
    """
    

class PhysicalAttribute(Base):
    """
    Physical value of a physical object, e.g. density, color, etc.
    """
    

class PhysicalAgent(Base):
    """
    A PhysicalObject that is capable of self-representing (conceptualizing) a Description in order to plan an Action. 
    A PhysicalAgent is a substrate for (actsFor) a Social Agent
    """
    

class BoxShape(Base):
    """
    A symmetrical shape, either solid or hollow, contained by six rectangles.
    """
    

class CylinderShape(Base):
    """
    A solid geometrical figure with straight parallel sides and a circular or oval cross section.
    """
    

class Configuration(Base):
    """
    A collection whose members are 'unified', i.e. organized according to a certain schema that can be represented by a Description.
    Typically, a configuration is the collection that emerges out of a composed entity: an industrial artifact, a plan, a discourse, etc.  
    E.g. a physical book has a configuration provided by the part-whole schema that holds together its cover, pages, ink. That schema, based on the individual relations between the book and its parts, can be represented in a reified way by means of a (structural) description, which is said to 'unify' the book configuration.
    """
    

class MeshShape(Base):
    """
    A solid geometrical figure described in a mesh file.
    """
    

class SixDPose(Base):
    """
    A point in three dimensional space, given as translation in a reference coordinate system, and an orientation of a coordinate system centered at that point relative to the reference coordinate system.
    """
    

class SphereShape(Base):
    """
    A round solid figure with every point on its surface equidistant from its centre.
    """
    

class Link(Base):
    ...
    

class Joint(Base):
    """
    An object that is used to articulate links in a kinematic structure.
    """
    

class DampingAttribute(Base):
    ...
    

class Inertia(Base):
    """
    A property of physical objects to continue the current motion or state of rest unless external force is applied.
    """
    

class JointAxis(Base):
    """
    The joint axis specified in the joint frame. This is the axis of rotation for revolute joints, the axis of translation for prismatic joints, and the surface normal for planar joints. The axis is specified in the joint frame of reference. Fixed and floating joints do not use the axis field.
    """
    

class JointLimits(Base):
    """
    Quantifies position, velocity and effort limit.
    """
    

class JointPosition(Base):
    """
    The position of a joint that moves connected links relative to each other.
    """
    

class JointReferencePositions(Base):
    """
    Reference joint positions indicating at which position a falling and raising edge are expected.
    """
    

class JointSoftLimits(Base):
    """
    The limits of a joint where the safety controller starts limiting the joint position.
    """
    

class ContinuousJoint(Base):
    """
    a continuous hinge joint that rotates around the axis and has no upper and lower limits.
    """
    

class HingeJoint(Base):
    """
    A joint that rotates along an axis.
    """
    

class FixedJoint(Base):
    """
    A joint that cannot move, designed to fixiate links.
    """
    

class MovableJoint(Base):
    """
    A joint where the two connected links can move relative to each other in some dimension.
    """
    

class FloatingJoint(Base):
    """
    A joint that allows motion for all 6 degrees of freedom.
    """
    

class JointAttribute(Base):
    """
    An attribute of a joint.
    """
    

class PlanarJoint(Base):
    """
    A joint that allows motion in a plane perpendicular to the axis.
    """
    

class PrismaticJoint(Base):
    """
    A sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
    """
    

class RevoluteJoint(Base):
    """
    a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
    """
    

