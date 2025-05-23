from owlready2 import Imp

from .dependencies import *


class World(Base):
    """
    The world could be the belief state of an agent about the world.
    """


class Floor(Base):
    """
    The floor of the world.
    """


class Milk(Base):
    """
    A milk carton.
    """


class Robot(Base):
    """
    The robot.
    """


class Cereal(Base):
    """
    A cereal box.
    """


class Kitchen(Base):
    """
    A kitchen.
    """


class Food(Base):
    ...


class Fruit(Food):
    ...


class Apple(Fruit):
    ...


class Environment(Base):
    ...


class Apartment(Environment):
    ...


class Cup(Base):
    ...


class Spoon(Base):
    ...


class Bowl(Base):
    ...


class PreferredGraspAlignment(Base):
    ...


class XAxis(Base):
    value = (1, 0, 0)


class YAxis(Base):
    value = (0, 1, 0)


class NoAlignment(Base):
    value = (0, 0, 0)


class Truthy(Base):
    value = True


class Falsy(Base):
    value = False


class Cabinet(Base):
    ...


class Washer(Base):
    ...


class Drawer(Base):
    ...


class Refrigerator(Base):
    ...


class Sink(Base):
    ...


class Door(Base):
    ...


class Cutting(Base):
    ...


class Pouring(Base):
    ...


class Handle(Base):
    ...


class Link(Base):
    ...


class PhysicalObject(Base):
    ...


class PouringTool(PhysicalObject):
    """
    The Tool that is used for pouring, can be cup, bottle, etc.
    """


class CuttingTool(PhysicalObject):
    """
    The tool used for cutting — such as a knife, bread-knife, or similar — depends on the specific task and object being processed.
    """


class MixingTool(PhysicalObject):
    """
    The tool used for mixing — such as a Spoon, Whisk, or similar — depends on the specific task and object being processed.
    """


class Agent(Base):
    ...


class Human(Base):
    ...


class Room(Base):
    ...


class Location(Base):
    ...


class Container(Base):
    ...


class Joint(Base):
    ...


class ContinuousJoint(Base):
    """
    A continuous hinge joint that rotates around an axis and has no upper and lower limits.
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


class PlanarJoint(Base):
    """
    A joint that allows motion in a plane perpendicular to an axis.
    """


class PrismaticJoint(Base):
    """
    A sliding joint that slides along an axis, and has a limited range specified by the upper and lower limits.
    """


class RevoluteJoint(Base):
    """
    A hinge joint that rotates along an axis and has a limited range specified by the upper and lower limits.
    """


class DesignedFurniture(Base):
    """
    An object used to make a room or building suitable for living or working.
    """


class Surface(Base):
    """
    The outside part or uppermost layer of something.
    """


class PhysicalTask(Base):
    """
    A task in which a PhysicalAgent affects some physical object.
    """


class Action(Base):
    """
    An Event with at least one Agent that isParticipantIn it, and that executes a Task that typically isDefinedIn a Plan, Workflow, Project, etc.
    """


class Event(Base):
    """
    Any physical, social, or mental process, event, or state.

    More theoretically, events can be classified in different ways, possibly based on 'aspect' (e.g. stative, continuous, accomplishement, achievement, etc.), on 'agentivity' (e.g. intentional, natural, etc.), or on 'typical participants' (e.g. human, physical, abstract, food, etc.).
    Here no special direction is taken, and the following explains why: events are related to observable situations, and they can have different views at a same time.
    If a position has to be suggested here anyway, the participant-based classification of events seems the most stable and appropriate for many modelling problems.

    (1) Alternative aspectual views

    Consider a same event 'rock erosion in the Sinni valley': it can be conceptualized as an accomplishment (what has brought a certain state to occur), as an achievement (the state resulting from a previous accomplishment), as a punctual event (if we collapse the time interval of the erosion into a time point), or as a transition (something that has changed from a state to a different one).
    In the erosion case, we could therefore have good motivations to shift from one aspect to another: a) causation focus, b) effectual focus, c) historical condensation, d) transition (causality).

    The different views refer to the same event, but are still different: how to live with this seeming paradox?
    A typical solution e.g. in linguistics (cf. Levin's aspectual classes) and in DOLCE Full (cf. WonderWeb D18 axiomatization) is to classify events based on aspectual differences. But this solution would create different identities for a same event, where the difference is only based on the modeller's attitude.
    An alternative solution is suggested here, and exploits the notion of (observable) Situation; a Situation is a view, consistent with a Description, which can be observed of a set of entities. It can also be seen as a 'relational context' created by an observer on the basis of a 'frame'. Therefore, a Situation allows to create a context where each particular view can have a proper identity, while the Event preserves its own identity.
    For example, ErosionAsAccomplishment is a Situation where rock erosion is observed as a process leading to a certain achievement: the conditions (roles, parameters) that suggest such view are stated in a Description, which acts as a 'theory of accomplishments'. Similarly, ErosionAsTransition is a Situation where rock erosion is observed as an event that has changed a state to another: the conditions for such interpretation are stated in a different Description, which acts as a 'theory of state transitions'.
    Consider that in no case the actual event is changed or enriched in parts by the aspectual view.

    (2) Alternative intentionality views

    Similarly to aspectual views, several intentionality views can be provided for a same Event. For example, one can investigate if an avalanche has been caused by immediate natural forces, or if there is any hint of an intentional effort to activate those natural forces.
    Also in this case, the Event as such has not different identities, while the causal analysis generates situations with different identities, according to what Description is taken for interpreting the Event.
    On the other hand, if the possible actions of an Agent causing the starting of an avalanche are taken as parts of the Event, then this makes its identity change, because we are adding a part to it.
    Therefore, if intentionality is a criterion to classify events or not, this depends on if an ontology designer wants to consider causality as a relevant dimension for events' identity.

    (3) Alternative participant views

    A slightly different case is when we consider the basic participants to an Event. In this case, the identity of the Event is affected by the participating objects, because it depends on them.
    For example, if snow, mountain slopes, wind, waves, etc. are considered as an avalanche basic participants, or if we also want to add water, human agents, etc., that makes the identity of an avalanche change.
    Anyway, this approach to event classification is based on the designer's choices, and more accurately mirrors lexical or commonsense classifications (see. e.g. WordNet 'supersenses' for verb synsets).

    Ultimately, this discussion has no end, because realists will keep defending the idea that events in reality are not changed by the way we describe them, while constructivists will keep defending the idea that, whatever 'true reality' is about, it can't be modelled without the theoretical burden of how we observe and describe it.
    Both positions are in principle valid, but, if taken too radically, they focus on issues that are only partly relevant to the aim of computational ontologies, which assist domain experts in representing a certain portion of reality according to their own assumptions and requirements.

    For this reason, in this ontology version of DOLCE, both events and situations are allowed, together with descriptions (the reason for the inclusion of the D&S framewrok in DOLCE), in order to encode the modelling needs, independently from the position (if any) chosen by the model designer.
    """


class Entity(Base):
    """
    Anything: real, possible, or imaginary, which some modeller wants to talk about for some purpose.
    """


class Task(Base):
    """
    An EventType that classifies an Action to be executed.
    For example, reaching a destination is a task that can be executed by performing certain actions, e.g. driving a car, buying a train ticket, etc.
    The actions to execute a task can also be organized according to a Plan that is not the same as the one that defines the task (if any).
    For example, reaching a destination could be defined by a plan to get on holidays, while the plan to execute the task can consist of putting some travels into a sequence.
    """


class RootLink(Base):
    ...


class Supporter(Base):
    """
    A physical object that supports another object.
    """
    ...


class SupportedObject(Base):
    """
    A physical object that is supported by another object.
    """
    ...
