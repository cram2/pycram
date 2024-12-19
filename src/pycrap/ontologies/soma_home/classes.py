from .dependencies import *


class Location(Base):
    """
    A role classifying a location of interest, often specified as a spatial relation between several objects, themselves usually classified by spatial relation roles.
    """
    

class Room(Base):
    """
    Space that can be occupied or where something can be done.
    """
    

class Entity(Base):
    """
    Anything: real, possible, or imaginary, which some modeller wants to talk about for some purpose.
    """
    

class Appliance(Base):
    """
    A device designed to perform a specific task, and that can be operated in some way.
    """
    

class BakedGood(Base):
    ...
    

class Blade(Base):
    """
    A flat cutting edge of an object used to cut through other objects, such as the blade of a ventilator cutting through the air.
    """
    

class Bottle(Base):
    """
    A container with a narrow neck used to store fluids.
    """
    

class Bowl(Base):
    """
    A round, deep object used to contain food or liquid.
    """
    

class Box(Base):
    """
    A cuboid-shaped container.
    """
    

class Bread(Base):
    ...
    

class BreakfastPlate(Base):
    """
    A smaller plate used for serving bread at the dining table with the main meal or to serve breakfast.
    """
    

class CerealBox(Base):
    """
    A box which holds, or at least is intended to usually hold, cereal flakes.
    """
    

class Cup(Base):
    """
    A round shaped container used for storing liquid and drinking out of it, typically has  a handle for grasping.
    """
    

class Cupboard(Base):
    """
    A piece of furniture for storing things.
    """
    

class Cutlery(Base):
    ...
    

class Kitchen(Base):
    """
    A designated room to perform the task of cooking.
    """
    

class CuttingTool(Base):
    ...
    

class Deposition(Base):
    """
    The disposition to support objects.
    """
    

class DesignedChair(Base):
    """
    A piece of furniture designed for one person to sit on.
    """
    

class DesignedComponent(Base):
    """
    An object designed to be part or element of a larger whole.
    """
    

class DesignedContainer(Base):
    """
    An item designed to be able to hold some other items, preventing their free movement and/or protecting them from outside influence. Containers may be used for storage, or to obtain control over items that are otherwise hard to manipulate directly (e.g. liquids).
    """
    

class DesignedFurniture(Base):
    """
    An object used to make a room or building suitable for living or working.
    """
    

class DesignedHandle(Base):
    """
    An item designed to fit well within a grasping hand, often attached to another item to enhance its manipulability.
    """
    

class DesignedTool(Base):
    """
    An item designed to enable some action, in which it will play an instrumental role.
    """
    

class DinnerPlate(Base):
    """
    A regular size plate for eating a main meals.
    """
    

class Dish(Base):
    ...
    

class Dishwasher(Base):
    """
    An appliance for washing dishes and cutlery automatically.
    """
    

class DishwasherTab(Base):
    """
    A solid detergent inserted into dishwashers.
    """
    

class Disposition(Base):
    """
    The tendency of an object (the bearer) to make certain events happen with others (the triggers).
    extrinsic
    """
    

class Door(Base):
    """
    A hinged, sliding, or revolving barrier at the entrance to a building, room, or vehicle, or in the frame of a cupboard.
    """
    

class Drawer(Base):
    """
    A storage compartment without a lid, made to slide horizontally in and out of a piece of furniture.
    """
    

class Fluid(Base):
    """
    A substance with a consistency such that it can flow or diffuse.
    """
    

class Fork(Base):
    """
    Cutlery with two or more prongs used for lifting food to the mouth or holding it when cutting.
    """
    

class Glass(Base):
    """
    A container made usually from glass or transparent plastic which is used to hold, typically cold, beverages.
    """
    

class Intrinsic(Base):
    """
    A physical quality that is independent of context.
    intrinsic
    """
    

class Jar(Base):
    """
    A container used for long-term storage of some liquid or viscous dish.
    """
    

class KitchenCabinet(Base):
    """
    A cupboard designed to be used in kitchen environments.
    """
    

class KitchenKnife(Base):
    """
    A tool designed to be used for kitchen related common tasks. Such as smearing a bread or cutting a cucumber.
    """
    

class Knife(Base):
    """
    An instrument composed of a blade fixed into a handle, used for cutting or as a weapon.
    """
    

class Lid(Base):
    """
    A removable or hinged cover for the top of a container.
    """
    

class MilkBottle(Base):
    """
    A bottle which holds, or at least is intended to usually hold, milk.
    """
    

class MilkPack(Base):
    """
    A pack which holds, or at least is intended to usually hold, milk.
    """
    

class Pack(Base):
    """
    A small cardboard or paper container.
    """
    

class Pan(Base):
    """
    A container used for cooking food.
    """
    

class Pancake(Base):
    ...
    

class PastaBowl(Base):
    """
    A bowl which holds, or at least is intended to usually hold, pasta.
    """
    

class Plate(Base):
    """
    A flat and usually circular object from which food is eaten or served.
    """
    

class Pot(Base):
    """
    A container used for cooking food.
    """
    

class Preference(Base):
    """
    A 'Preference' is a 'Quality' of an 'Agent' that orders 'Situation's by some heuristic based on the happiness, satisfaction, gratification, morality, enjoyment, and utility (see alse https://en.wikipedia.org/wiki/Preference) they provide to their bearing Agent.
    
    The pattern is as follows: A 'Preference' 'is described by' a 'Predilection', which also 'describes' an 'Order' that 'orders' 'Order item's that contain only 'Situation's. The 'Situation's then are modeled according to what the preference entails.
    
    That a 'Preference' orders 'Situation's might be unintuitive, but makes the model very general. A few examples:
    
    Example 1
    
    "Peter likes coffee and dislikes tea".
    Here, between different hypothetical situations where he plays the role of a performer in a drinking task, Peter prefers the situations in which role of the drunken object is played by some coffee (vs. some tea). Note that the coffe and tea are hypothetical objects as well and could, for example, be represented via reified Concepts.
    
    Example 2
    
    "Would you like this pot of coffee, or this pot of tea, Peter?"
    Here, as opposed to Example 1, the pot of coffee and the pot of tea are not hypothetical, but concrete.
    
    Example 3
    
    "Would you like this pot of coffee, or should I brew you some tea?"
    Here, the pot of coffee is concrete and the tea is not.
    
    Example 4
    
    Situations are not restricted to Tasks; other event types are possible as well.
    For example, Peter might prefer the Containment State of a tiger being inside a cage vs. the Containment State of the tiger being outside of the cage.
    """
    

class Rack(Base):
    """
    A frame for holding or storing things.
    """
    

class Refrigerator(Base):
    """
    An appliance which is artificially kept cool and used to store food and drink.
    """
    

class SaladBowl(Base):
    """
    A bowl which holds, or at least is intended to usually hold, salad.
    """
    

class SaltShaker(Base):
    """
    A shaker which holds, or at least is intended to usually hold, salt.
    """
    

class Shaker(Base):
    """
    A container used for storing fine grained substances such as salt or paper.
    """
    

class Sink(Base):
    """
    A large bowl used to direct the flow of liquid into a drain, typically has a tap used to fill the sink with water, and a plug used to block the drain.
    """
    

class Sofa(Base):
    """
    A comfortable seat for two or more people to sit on.
    """
    

class Spatula(Base):
    ...
    

class Spoon(Base):
    """
    An eating or cooking implement consisting of a small shallow bowl with a relatively long handle.
    """
    

class Table(Base):
    """
    A piece of furniture with a flat top and one or more legs.
    """
    

class TrashContainer(Base):
    """
    An item designed to hold trash. Typically equipped with a mechanism to close or open it, to prevent smells from the trash from propagating out and insects and other pests to get in.
    """
    

class WaterGlass(Base):
    """
    A glass which holds, or at least is intended to usually hold, water. Also affords drinking from it.
    """
    

class WineBottle(Base):
    """
    A bottle which holds, or at least is intended to usually hold, wine.
    """
    

class WineGlass(Base):
    """
    A glass which holds, or at least is intended to usually hold, wine. Also affords drinking from it.
    """
    

class SocialObject(Base):
    """
    Any Object that exists only within some communication Event, in which at least one PhysicalObject participates in. 
    In other words, all objects that have been or are created in the process of social communication: for the sake of communication (InformationObject), for incorporating new individuals (SocialAgent, Place), for contextualizing or intepreting existing entities (Description, Concept), or for collecting existing entities (Collection).
    Being dependent on communication, all social objects need to be expressed by some information object (information objects are self-expressing).
    """
    

class DesignedSubstance(Base):
    ...
    

class Action(Base):
    """
    An Event with at least one Agent that isParticipantIn it, and that executes a Task that typically isDefinedIn a Plan, Workflow, Project, etc.
    """
    

class State(Base):
    """
    States are stative and homeomeric events.
    
    For stative events, the mereological sum of two instances has the same type as both instances. This is, for example, the state of sitting on a chair, or the process of a pendulum swinging around.
    
    The difference between states and processes is that states are, in addition, homeomeric, and processes are not.  This means that, when considering time slices  of an event, for states, these time slices always have the same type as the state, but for processes this is not the case.
    """
    

class NaturalPerson(Base):
    """
    A person in the physical commonsense intuition: 'have you seen that person walking down the street?'
    """
    

class DesignedArtifact(Base):
    """
    A PhysicalArtifact that is also described by a Design. This excludes simple recycling or refunctionalization of natural objects. Most common sense 'artifacts' can be included in this class: cars, lamps, houses, chips, etc.
    """
    

class BiologicalObject(Base):
    ...
    

class Substance(Base):
    """
    Any PhysicalBody that has not necessarily specified (designed) boundaries, e.g. a pile of trash, some sand, etc. 
    In this sense, an artistic object made of trash or a dose of medicine in the form of a pill would be a FunctionalSubstance, and a DesignedArtifact, since its boundaries are specified by a Design; aleatoric objects that are outcomes of an artistic process might be still considered DesignedArtifact(s), and Substance(s).
    """
    

class Collection(Base):
    """
    Any container for entities that share one or more common properties. E.g. "stone objects", "the nurses", "the Louvre Aegyptian collection", all the elections for the Italian President of the Republic. 
    A collection is not a logical class: a collection is a first-order entity, while a class is second-order.
    A collection is neither an aggregate of its member entities (see e.g. ObjectAggregate class).
    """
    

class Plan(Base):
    """
    A Description having an explicit Goal, to be achieved by executing the plan
    """
    

class Compartment(Base):
    ...
    

class PancakeMix(Base):
    ...
    

class Tableware(Base):
    ...
    

class Affordance(Base):
    """
    A relation between an object (the bearer) and others (the triggers) that describes the disposition of the bearer to be involved in an action execution that also involves some trigger object.
    """
    

class Concept(Base):
    """
    A Concept is a SocialObject, and isDefinedIn some Description; once defined, a Concept can be used in other Description(s). If a Concept isDefinedIn exactly one Description, see the LocalConcept class.
    The classifies relation relates Concept(s) to Entity(s) at some TimeInterval
    """
    

class Task(Base):
    """
    An EventType that classifies an Action to be executed. 
    For example, reaching a destination is a task that can be executed by performing certain actions, e.g. driving a car, buying a train ticket, etc. 
    The actions to execute a task can also be organized according to a Plan that is not the same as the one that defines the task (if any). 
    For example, reaching a destination could be defined by a plan to get on holidays, while the plan to execute the task can consist of putting some travels into a sequence.
    """
    

class Role(Base):
    """
    A Concept that classifies an Object
    """
    

class Setpoint(Base):
    """
    Classifies some dedicated goal region.
    """
    

class Answer(Base):
    """
    A role that is played by an Information Realization answering some query.
    """
    

class Message(Base):
    """
    A message is a discrete unit of communication intended by the source for consumption by some recipient or group of recipients (Source: https://en.wikipedia.org/wiki/Message).
    
    Note that the Role Message classifies the Information Realization, not the content.
    """
    

class PhysicalObject(Base):
    """
    Any Object that has a proper space region. The prototypical physical object has also an associated mass, but the nature of its mass can greatly vary based on the epistemological status of the object (scientifically measured, subjectively possible, imaginary).
    """
    

class Description(Base):
    """
    A Description is a SocialObject that represents a conceptualization. 
    It can be thought also as a 'descriptive context' that uses or defines concepts in order to create a view on a 'relational context' (cf. Situation) out of a set of data or observations. 
    For example, a Plan is a Description of some actions to be executed by agents in a certain way, with certain parameters; a Diagnosis is a Description that provides an interpretation for a set of observed entities, etc.
    Descriptions 'define' or 'use' concepts, and can be 'satisfied' by situations.
    """
    

class EventType(Base):
    """
    A Concept that classifies an Event . An event type describes how an Event should be interpreted, executed, expected, seen, etc., according to the Description that the EventType isDefinedIn (or used in)
    """
    

class Parameter(Base):
    """
    A Concept that classifies a Region; the difference between a Region and a Parameter is that regions represent sets of observable values, e.g. the height  of a given building, while parameters represent constraints or selections on observable values, e.g. 'VeryHigh'. Therefore, parameters can also be used to constrain regions, e.g. VeryHigh on a subset of values of the Region Height applied to buildings, or to add an external selection criterion , such as measurement units, to regions, e.g. Meter on a subset of values from the Region Length applied to the Region Length applied to roads.
    """
    

class ProcessType(Base):
    """
    An EventType that classifies Processes.
    """
    

class Quality(Base):
    """
    Any aspect of an Entity (but not a part of it), which cannot exist without that Entity. For example, the way the surface of a specific PhysicalObject looks like, or the specific light of a place at a certain time, are examples of Quality, while the encoding of a Quality into e.g. a PhysicalAttribute should be modeled as a Region. 
    From the design viewpoint, the Quality-Region distinction is useful only when individual aspects of an Entity are considered in a domain of discourse. 
    For example, in an automotive context, it would be irrelevant to consider the aspects of car windows for a specific car, unless the factory wants to check a specific window against design parameters (anomaly detection). 
    On the other hand, in an antiques context, the individual aspects for a specific piece of furniture are a major focus of attention, and may constitute the actual added value, because the design parameters for old furniture are often not fixed, and may not be viewed as 'anomalies'.
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
    

class OrderedElement(Base):
    """
    A 'Singleton' of an entity that 'is ordered by' some 'Order'. An 'Order item' can only 'precede' or 'follow' another 'Order item', encoding the sortation of the entities contained within the 'Order items'. Different 'Order's need to use different 'Order item's.
    """
    

class InformationObject(Base):
    """
    A piece of information, such as a musical composition, a text, a word, a picture, independently from how it is concretely realized.
    """
    

class MotionProcess(Base):
    """
    A class that classifies motion processes. This class is used to represent any process involving motion, encompassing a wide range of activities such as walking, running, jumping, and any other form of physical movement.
    """
    

class Motion(Base):
    """
    An EventType that classifies motion Processes.
    Motion type
    """
    

class System(Base):
    """
    A system is a group of interacting or interrelated elements that act according to a set of rules to form a unified whole.
    
    From Wikipedia: https://en.wikipedia.org/wiki/System
    """
    

class Region(Base):
    """
    Any region in a dimensional space (a dimensional space is a maximal Region), which can be used as a value for a quality of an Entity . For example, TimeInterval, SpaceRegion, PhysicalAttribute, Amount, SocialAttribute are all subclasses of Region. 
    Regions are not data values in the ordinary knowledge representation sense; in order to get patterns for modelling data, see the properties: representsDataValue and hasDataValue
    """
    

class Binding(Base):
    """
    A Relation between Roles/Parameters and their fillers that holds in a particular descriptive context, such as a Workflow or Narrative.
    
    It covers two conceptually related, but somewhat ontologically distinct situations:
    
    -- a binding between two Roles, or two Parameters, with the meaning that in the particular descriptive context where the Binding holds, a filler for one Role/Parameter is also a filler for the other
    -- a binding between a Role/Parameter and an Entity able to fill that Role/Parameter, with the meaning that in the particular descriptive context where the Binding holds, the Entity fills the Role/Parameter.
    
    Note: in the second case, the Entity filling the Role/Parameter may be a Role or Parameter itself. This however does NOT reduce to the first case. Consider these examples:
    
    -- (first situation) The accountant is also the lawyer. In this case, there are two roles, and there is a person filling both of them. This is a factual, role-role binding.
    -- (second situation, linking to a generic Entity) The accountant is Bob. In this case, there is a factual, role-filler binding asserting who fills the accountant role.
    -- (second situation, linking to a role) The newly opened job is accountant. In this case, there is a factual, role-filler binding asserting that some role is filled by another, without making any assertion about the filler of this second role. It is not known, and not important, whether an accountant exists at this time.
    
    There is a further, orthogonal distinction made between:
    -- factual: the Binding is asserted to hold in the descriptive context
    -- counterfactual: the Binding is used to express conditions in the descriptive context. A counterfactual Binding is not meant as an assertion that the Binding actually holds.
    """
    

class Joint(Base):
    """
    An object that is used to articulate links in a kinematic structure.
    """
    

class Color(Base):
    """
    The color of an object. Color regions encode the color value in some space such as RGB or HSV, and may further be used to classify the color as red, dark, etc. The color of an object may have different facets, e.g. a red and blue color.
    """
    

class Object(Base):
    """
    Any physical, social, or mental object, or a substance. Following DOLCE Full, objects are always participating in some event (at least their own life), and are spatially located.
    """
    

class ExecutionStateRegion(Base):
    """
    A region containing labels that describe different states in the evolution/completion of a task execution.
    """
    

class Feature(Base):
    """
    Features are 'parasitic' entities that only exist insofar their host exists. Typical examples are holes, bumps, boundaries, or spots of color.
    """
    

class Workflow(Base):
    """
    A Plan that defines Role(s), Task(s), and a specific structure for tasks to be executed, usually supporting the work of an Organization
    """
    

class FrictionAttribute(Base):
    """
    The resistance that one surface or object encounters when moving over another.
    """
    

class SituationTransition(Base):
    """
    A transition between two situations, usually brought about by the Action of some Agent.
    """
    

class Situation(Base):
    """
    A view, consistent with ('satisfying') a Description, on a set of entities. 
    It can also be seen as a 'relational context' created by an observer on the basis of a 'frame' (i.e. a Description). 
    For example, a PlanExecution is a context including some actions executed by agents according to certain parameters and expected tasks to be achieved from a Plan; a DiagnosedSituation is a context of observed entities that is interpreted on the basis of a Diagnosis, etc.
    Situation is also able to represent reified n-ary relations, where isSettingFor is the top-level relation for all binary projections of the n-ary relation. 
    If used in a transformation pattern for n-ary relations, the designer should take care of adding (some or all) OWL2 keys, corresponding to binary projections of the n-ary, to a subclass of Situation. Otherwise the 'identification constraint' (Calvanese et al., IJCAI 2001) might be violated.
    """
    

class NonmanifestedSituation(Base):
    """
    A Situation which does not manifest in any event.
    
    The main use case for this is to represent expectations that are not met, e.g. unfulfilled post-conditions of an action. An action with unmet postconditions is then a failure.
    
    Because of open world semantics of DL, the default assumption for a Situation individual with no "manifests in" relations is simply that we don't know yet whether that Situation is manifested and if so by what Event.
    
    As such, an explicit assertion is needed to make a Situation a nonmanifested one: either declare that individual's type NonmanifestedSituation, or assert that it has 0 manifestsIn relations.
    """
    

class JointLimit(Base):
    """
    The physical limits of a joint.
    """
    

class JointState(Base):
    """
    The state of a joint in terms of position, velocity of the joint and effort applied to it.
    """
    

class Localization(Base):
    """
    The localization of an object. The region of this quality encodes values to localize the object in a dimensional space, e.g. Euclidean positions that localize the object in Euclidean space.
    """
    

class MassAttribute(Base):
    """
    The quantity of matter which a body contains, as measured by its acceleration under given force or by the force exerted on it by a gravitational field.
    """
    

class NetForce(Base):
    """
    The accumulated force acting upon an object.
    """
    

class Succedence(Base):
    """
    A relation that holds in some descriptive context such as a Workflow, between two TaskInvocations belonging to that same Workflow. It means that one task invocation should follow the other.
    
    Note: a successor relation implemented as an OWL object property is sometimes enough, but not in general; in particular, when there are conditions imposed on the succession.
    
    As a result, a reification pattern was applied here.
    Note: it is possible for a Succedence relation to exist between a TaskInvocation and itself; in this case, both the hasPredecessor and hasSuccessor roles are filled by the same TaskInvocation individual.
    
    Care must be taken with this however, as a natural interpretation of this situation is an infinite loop.
    """
    

class Agent(Base):
    """
    Additional comment: a computational agent can be considered as a PhysicalAgent that realizes a certain class of algorithms (that can be considered as instances of InformationObject) that allow to obtain some behaviors that are considered typical of agents in general. For an ontology of computational objects based on DOLCE see e.g. http://www.loa-cnr.it/COS/COS.owl, and http://www.loa-cnr.it/KCO/KCO.owl.
    Any agentive Object , either physical (e.g. a whale, a robot, an oak), or social (e.g. a corporation, an institution, a community).
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
    

class SoftwareInstance(Base):
    """
    A Software instance is an entity to represent the agent that emerges from and while executing software: Some object, that can perform actions and communicate via some interfaces.
    In this view, we see the of an Agent required intentionality its intentionality as bestowed upon the Software instance from the agents who started the program or gave an input (e.g., via a mouse click) to achieve some goal.
    
    Another apporach might be to not model this entity at all and only see Execution of Software as a Process (see, e.g., https://en.wikipedia.org/wiki/Execution_(computing)). However, this would complicate modeling communication between running Software processes.
    """
    

class SpaceRegion(Base):
    """
    Any Region in a dimensional space that is used to localize an Entity ; i.e., it is not used to represent some characteristic (e.g. it excludes time intervals, colors, size values, judgment values, etc.). Differently from a Place , a space region has a specific dimensional space.
    """
    

class StateType(Base):
    """
    An EventType that classifies States.
    """
    

class Relation(Base):
    """
    Relations are descriptions that can be considered as the counterpart of formal relations (that are included in the FormalEntity class).
    For example, 'givingGrantToInstitution(x,y,z)' with three argument types: Provider(x),Grant(y),Recipient(z), can have a Relation counterpart: 'GivingGrantToInstitution', which defines three Concept instances: Provider,Grant,Recipient.
    Since social objects are not formal entities, Relation includes here any 'relation-like' entity in common sense, including social relations.
    """
    

class PhysicalEffector(Base):
    """
    A functional part belonging to an Agent and which allows that Agent to act upon its surroundings.
    """
    

class QueryingTask(Base):
    """
    An Illocutionary act where the Sender of the Message does so to trigger the Receiver to return some information that is specified within the content of the Message.
    """
    

class Design(Base):
    """
    A Description of the Situation, in terms of structure and function, held by an Entity for some reason.
    A design is usually accompanied by the rationales behind the construction of the designed Entity (i.e. of the reasons why a design is claimed to be as such). For example, the actual design (a Situation) of a car or of a law is based on both the specification (a Description) of the structure, and the rationales used to construct cars or laws.
    While designs typically describe entities to be constructed, they can also be used to describe 'refunctionalized' entities, or to hypothesize unknown functions. For example, a cradle can be refunctionalized as a flowerpot based on a certain home design.
    """
    

class MotionDescription(Base):
    """
    A Description of a Motion process, e.g. moving the arm into a specific location
    """
    

class Order(Base):
    """
    An 'Order' sorts two or more 'Order item's via the relations 'precedes' and 'follows'.
    """
    

class Transient(Base):
    """
    Objects may undergo changes during Processes; however, while the change Process is in operation, one cannot strictly say either the input of the Process still exists, nor that the result exists yet.
    
    A prototypical example is making a pancake on a heater. When PancakeMix is put on the hot plate, it ceases to be PancakeMix-- already, the chemical structure of the substance gets altered-- however it is only after sufficient heating that this object becomes a Pancake.
    
    Transients are the objects undergoing such change processes; they are no longer the initial objects fed into the process, nor are they the objects produced as results.
    
    Instead, a Transient transitionsFrom some initial Object that was fed into a change Process. Typically, a Transient may transitionTo some resulting Object (though not always, some processes simply destroy objects).
    
    It is also possible that a Transient transitionsBack to the initial object. An example is the catalyst in a chemical reaction; another example is a loaf of bread after a slice has been cut off.
    """
    

class ColorRegion(Base):
    """
    Encodes the color of an object.
    """
    

class ForceAttribute(Base):
    """
    The value of a force dynamical characteristic. An example is the force exerted on another object when pushing it.
    """
    

class PhysicalAttribute(Base):
    """
    Physical value of a physical object, e.g. density, color, etc.
    """
    

class APISpecification(Base):
    """
    An application programming interface (API) is a way for two or more computer programs to communicate with each other. It is a type of software interface, offering a service to other pieces of software. A document or standard that describes how to build or use an API is called an API specification.
    
    Source: https://en.wikipedia.org/wiki/API
    """
    

class InterfaceSpecification(Base):
    """
    The Specification of an Interface between software, computer hardware, peripheral devices, humans and combinations of these.
    
    Source: https://en.wikipedia.org/wiki/Interface_(computing)
    """
    

class AbductiveReasoning(Base):
    """
    A task in which the Agent proceeds from some set of statements about a world, and attempts to obtain an explanation for these statements. This explanation is often an inferred cause, such as a final cause or intention. Further, it is often required that there be some guarantees that the explanation produced by AbductiveReasoning have some desirable property, such as being the simplest or most likely given the set of statements to explain.
    """
    

class Reasoning(Base):
    """
    A Mental task in which an Agent endeavours to obtain new knowledge from knowledge it already possesses.
    todo: a taxonomy of reasoning is not trivial. Classical theory distinguishes Deductive, Inductive, and with a stretch Abductive reasoning. However, modern practice distinguishes other categories that overlap with these, e.g. Probabilistic and Non-monotonic.
    
    Both Abductive and Inductive inference may, and often do, use Probabilistic methods. Probabilistic inference is, by its nature, most opposed to Deductive inference which, classically, requires logical certainty.
    
    Any of the Abductive/Deductive/Inductive triad can be further affected by the Monotone/Non-monotone distinction. There are preferences (Inductive and Abductive reasoning is probably most often non-monotonic; most of Deductive reasoning is probably done in monotonic formalisms), but it is certainly the case that, e.g., non-monotone Deduction is possible.
    
    Note, this classification has nothing to do with reasoning domain (e.g. SpatialReasoning, TemporalReasoning, ...) and merely with techniques/logical-mathematical underpinnings.
    """
    

class Accessor(Base):
    """
    A role classifying an object used to gain access to some other entity.
    """
    

class Instrument(Base):
    """
    An object used to carry out the event.
    """
    

class Accident(Base):
    """
    An Event for which causes are unknown and/or considered irrelevant. This is true also for "final causes" (that is, intentions) of Agents participating in the Accident: it is not the intentions of these Agents to bring about the Accident.
    
    Note a distinction between this definition and some informal, everyday uses of "accident" which require a causal structure and responsibility to be ascertained. An accident, in the informal sense, may require an explanation as to who made a mistake in bringing about the event; a "traffic accident", where we want to know who's responsible, is an example of this.
    
    Such an event does NOT fall under the definition of Accident here. An example of Accident would be a fair coin landing Heads: the causal chain for why this exact face landed is not important, all that matters is the brute fact that the coin landed Heads.
    also think about "mistakes": (Mihai:) yes, but consider whether those might qualify as Situations. Likewise for Accidents, actually.
    """
    

class ActionExecutionPlan(Base):
    """
    idea: steps in workflows assert that they are defined by action execution plans.
    links role and parameter fillers to e.g. slots in a data structure
    """
    

class Actuating(Base):
    """
    Tasks where the goal is to move an object.
    
    Usually, an agent will use their prehensile effectors, ie. hands, for this purpose, so there is a lot of conceptual overlap between Actuating and Manipulating.
    
    However, these categories are nonetheless distinguished in that there are more ways to actuate objects than simply manipulating them; for example, some tool like a net or frying pan might be used to catch an object.
    
    Another way to look at the difference between Actuating and Manipulating is in what they "profile", ie. focus on as important.
    
    For Actuating, it is the object's motion that is paramount.
    
    For Manipulating, it is the movement of the hand(s) and the change in functional relationships (such as kinematic control) between the hand(s) and the manipulated object(s).
    todo: think about use of tools. force events are generated between artifacts then. also not only the tool would be the 'salient artifact' here.
    """
    

class PhysicalTask(Base):
    """
    A task in which a PhysicalAgent affects some physical object.
    """
    

class AestheticDesign(Base):
    """
    A design that describes an aesthetic quality of an object.
    
    Aesthetics is the philosophical study of beauty and taste. The term stems from the Greek word 'aisthetikos', meaning 'of sense perception', and is related to the study of sensory values. From design point of view, aesthetics refers to the visual attractiveness of an object. Visual aesthetics have these key elements: Color, Shape, Pattern, Line, Texture, Visual weight, Balance, Scale, Proximity and Movement.
    """
    

class AffordsBeingSitOn(Base):
    """
    A reified relation between a furniture-like object (the bearer), the sitting task it affords, and an agent that can assume a sitting pose (trigger).
    """
    

class CanBeSatOn(Base):
    """
    The disposition to support an agent being in a sitting pose. Typically provided by chairs, stools, etc.
    """
    

class SittingDestination(Base):
    """
    The destination of an agent assuming a sitting pose. Typically a chair or stool would fill this role.
    """
    

class CanSit(Base):
    """
    The capability of an agent to assume a sitting pose.
    """
    

class AgentRole(Base):
    """
    A role classifying an Agent responsible for performing an Action.
    
    The entity playing an AgentRole is endowed with sentience and the capacity to deliberately choose actions in pursuit of goals. This distinguishes Agents from other causes that could bring an event about.
    """
    

class Sitting(Base):
    """
    A task where an agents assumes a sitting pose.
    """
    

class CausativeRole(Base):
    """
    A role classifying objects that are responsible in bringing about an event.
    
    The paradigmatic example is the Agent performing an Action -- the Agent is the effective cause of the Action it performs. However, not all objects causing events are agents.
    """
    

class Agonist(Base):
    """
    A role that classifies entities with a tendency to either cause an alteration or to preserve some state.
    """
    

class Patient(Base):
    """
    A role classifying an object that undergoes/is the primary object affected by the event.
    """
    

class Algorithm(Base):
    """
    An Algorithm is a finite sequence of well-defined instructions, typically used to solve a class of specific problems or to perform a computation.
    
    From Wikipedia: https://en.wikipedia.org/wiki/Algorithm
    """
    

class Alteration(Base):
    """
    A process by which an aspect of some object changes such as ice cream melting in the sun.
    """
    

class AlterativeInteraction(Base):
    """
    A force interaction where the agonist has the tendency to set another object into motion. An example is 'opening a door' where some object interacts with the door such that it moves out of its frame.
    """
    

class ForceInteraction(Base):
    """
    Classifies events in which two entities interact with each other with a reference to force. One of the entities, the agonist, has a tendency to either set the other entity (the antagonist) into motion, or to keep it still under the influence of some other force. The tendency only manifests in case the agonist is the "stronger entity".
    """
    

class PreservativeInteraction(Base):
    """
    A force interaction where the agonist has a tendency to keep another object still. An example is 'holding a door closed' where some object interacts with the door to neutralize forces that could set the door into motion.
    """
    

class AlteredObject(Base):
    """
    An object undergoing modifications.
    """
    

class Amateurish(Base):
    """
    A description of amateurish behavior.
    """
    

class AnsweringTask(Base):
    """
    An Illocutionary act where the Sender emits some Message to the Receiver as a reaction to some previous Communication task where the Roles where switched, i.e., the Sender (Receiver) of the Answering task has been the Sender (Sender) for the cause.
    """
    

class CommandingTask(Base):
    """
    An Illocutionary act where the Sender emits some Message with the intent to cause the Receiver to perform some action.
    """
    

class IllocutionaryTask(Base):
    """
    A task which is executed by a Locution action: A Locution is what was said and meant, Illocution is what was done.
    
    When somebody says "Is there any salt?" at the dinner table, the illocutionary act is a request: "please give me some salt" even though the locutionary act (the literal sentence) was to ask a question about the presence of salt.
    
    Source: https://en.wikipedia.org/wiki/Illocutionary_act
    """
    

class Antagonist(Base):
    """
    A role that classifies entities that are opposed to the tendency of some agonist.
    """
    

class Approaching(Base):
    """
    A process type to classify motions by which a body approaches some object or location.
    """
    

class Locomotion(Base):
    """
    Conceptually related to Navigation, but distinguishable from it because of the profile, ie. the focus of the task.
    
    Navigation is about reaching some goal.
    
    Locomotion is concerned more with the actual motion.
    The Agent repositions their body in the environment.
    """
    

class ArchiveFile(Base):
    """
    An archive file is a computer file that is composed of one or more files along with metadata.
    
    Source: https://en.wikipedia.org/wiki/Archive_file
    """
    

class ArchiveText(Base):
    """
    An Archive is a Structured Text that is composed of one or more Structured Texts along with metadata. Archives are used to collect multiple texts together into a single text for easier portability and storage as Archive Files, or simply to compress text to use less storage space as Computer Files. Archive often store directory structures, error detection and correction information, arbitrary comments, and sometimes use built-in encryption.
    
    Source: https://en.wikipedia.org/wiki/Archive_file
    """
    

class DigitalFile(Base):
    """
    Any file that exists as a digital resource (but not its content), e.g., a text file actually laying on some hard drive, but not the contained text.
    """
    

class ArchiveFormat(Base):
    """
    An Archive Format is the file format of an archive file.
    
    Source: https://en.wikipedia.org/wiki/Archive_file#Archive_formats
    """
    

class FileFormat(Base):
    """
    A File Format is a standard way that information is encoded for storage in a computer file. It specifies how bits are used to encode information in a digital storage medium.
    
    From Wikipedia: https://en.wikipedia.org/wiki/File_format
    """
    

class FileConfiguration(Base):
    """
    A configuration whose members are all files. Used to model, e.g., concrete collections of zip- and jar-files (and so on).
    """
    

class StructuredText(Base):
    """
    Any Text that adheres to some rules that are in any way more specific than natural language and that cannot be made sense of without knowing said rules.
    """
    

class AreaSurveying(Base):
    """
    A task in which an Agent uses its perception apparatus to gain information about some location.
    """
    

class Perceiving(Base):
    """
    A task in which the Agent gathers and interprets sensor information about its surroundings.
    """
    

class Arm(Base):
    """
    A limb used to reach for objects.
    """
    

class Limb(Base):
    """
    An arm or leg of an embodied agent.
    """
    

class Armchair(Base):
    """
    A more comfortable chair that includes sides to support arms.
    """
    

class Arranging(Base):
    """
    A task in which an Agent places a collection of objects at some set of relative poses to each other.
    """
    

class Constructing(Base):
    """
    A task in which an Agent creates a new physical object.
    """
    

class ArtificialAgent(Base):
    """
    A physical object with artificial characteristics, which can perform actions to achieve desired goals, and typically has sensors and/or actuators.
    
    There can be non-physical artificial agents such as software programs but they are not considered here in the scope of artificial agent.
    """
    

class PhysicalAgent(Base):
    """
    A PhysicalObject that is capable of self-representing (conceptualizing) a Description in order to plan an Action. 
    A PhysicalAgent is a substrate for (actsFor) a Social Agent
    """
    

class Assembling(Base):
    """
    A task in which an Agent connects some objects such that they form a cohesive whole, and which also imposes constraints on the objects' relative motions. Often, the objects that make up an assemblage can also be separated again.
    """
    

class AssertionTask(Base):
    """
    An Illocutionary Act where the Sender emits some Message with the intent to change what the Receiver believes to be true in some context. Often, assertions are of facts about the real world, but this need not be the case. Assertions can communicate what someone believes, or refer to a world that is entirely fictional. In all these cases however, assertions are intended to update the listener's model (of the real world, or of the speaker's beliefs, or of the fictional world etc.).
    """
    

class DeclarativeClause(Base):
    """
    A clause which makes an assertion or declaration.
    """
    

class AssumingArmPose(Base):
    """
    A task by which an Agent arranges one/some/all of its arms according to some configuration.
    """
    

class AssumingPose(Base):
    """
    A task by which an Agent arranges its body, or part of it, according to some configuration.
    """
    

class AttentionShift(Base):
    """
    A mental task in which the executing Agent shifts his attention from some Information to another.
    """
    

class MentalTask(Base):
    """
    A Task classifying some MentalAction, that is, an Action through which an Agent manipulates representations stored in its own cognition.
    """
    

class AvoidedObject(Base):
    """
    An object that is avoided.
    """
    

class Avoiding(Base):
    """
    A task in which an Agent moves so as to not enter or pass through a location.
    """
    

class Navigating(Base):
    """
    A task in which an Agent moves through space so as to arrive at some location, follow some path, or increase its distance from some location or other entity. Often, navigation involves finding paths around obstacles and forbidden areas.
    """
    

class Barrier(Base):
    """
    A role classifying an object used to prevent others from entering or leaving a restricted space or group.
    """
    

class Restrictor(Base):
    """
    A role classifying an object used to deny access to some other entity.
    """
    

class BedsideTable(Base):
    """
    A small table typically located next to a bed.
    """
    

class BehavioralDiagnosis(Base):
    """
    A diagnosis of how a system interacts with its world.
    """
    

class BeneficiaryRole(Base):
    """
    A role classifying an agent for whose benefit an action is performed.
    """
    

class GoalRole(Base):
    """
    A role classifying objects that constitute the goal of an action.
    """
    

class CounterfactualBinding(Base):
    """
    CounterfactualBindings are used to express conditions:
    
    -- that two roles share a filler (RoleRoleBindings);
    -- that the filler of a role is a particular entity (RoleFillerBindings). This is typically the case when the filler of the role may be one of a few constants, as is the case with the execution status of a robot task.
    
    TODO: for the robot workflows we are likely to need at the start, testing equality of fillers is enough. In the future, we may want to replace that with unifiability of fillers.
    """
    

class FactualBinding(Base):
    """
    FactualBindings are used in a workflow to assert that:
    
    -- task- or workflow-defined roles share fillers (RoleRoleBindings). Example, the instrument of a cutting task may be the actee of a grasping task;
    -- task- or workflow-defined roles are filled by a particular entity (RoleFillerBindings). This is typically the case when roles, and especially parameters, can be assigned to constants. Example, the waiting time to cook soft eggs is 3 minutes.
    """
    

class RoleFillerBinding(Base):
    """
    A binding that connects a role to a particular filler.
    """
    

class RoleRoleBinding(Base):
    """
    A binding that asserts that two roles have the same filler.
    """
    

class Blockage(Base):
    """
    The disposition of an object (the barrier) to prevent others from accessing, leaving, or seeing a restricted space, or group.
    """
    

class BlockedObject(Base):
    """
    An object that is blocked from accessing something.
    """
    

class BodyMovement(Base):
    """
    Motion described in terms of how parts of an Agent's body move.
    
    As such, this concept can be applied only to motions involving some PhysicalAgent, or body parts of a PhysicalAgent.
    """
    

class Boiling(Base):
    """
    In each instance of this collection some liquid matter is raised to its boilingPoint and is thereby changed from being in the Liquid-StateOfMatter to being in the Gaseous-StateOfMatter.
    """
    

class Vaporizing(Base):
    """
    Some material transitions from a liquid to a gaseous phase.
    """
    

class Crockery(Base):
    """
    An object designed to contain food to store, cook, eat, or serve it.
    """
    

class BoxShape(Base):
    """
    A symmetrical shape, either solid or hollow, contained by six rectangles.
    """
    

class BreadKnife(Base):
    """
    A knife to cut bread.
    """
    

class Building(Base):
    """
    A structure with a roof and walls.
    """
    

class CanCut(Base):
    """
    The disposition of an object (the tool) to cut other objects. Such as a knife has cutting ability to cut a cucumber into pieces.
    """
    

class Variability(Base):
    """
    The disposition of an object (the tool) to change some aspect of others.
    """
    

class Cutter(Base):
    """
    A role to classify an object used to cut other objects. Usually should poses sharpness as a quality. Execptions are not considered in this context. Such as a wind, water, or other natural agents cutting(eroding) the rocks.
    """
    

class CutObject(Base):
    """
    An object being cut down into pieces.
    """
    

class Capability(Base):
    """
    Capability
    The disposition of an agent to bring about certain effects, and to achieve certain goals.
    The tendency of an object (the bearer) to be able to perform certain tasks together with others (the triggers) and in which the Bearer is the executor of the associated Task and will therefore usually be an Agent.
    """
    

class Capacity(Base):
    """
    The maximum amount an object can contain.
    """
    

class Carafe(Base):
    """
    A container used to hold and pour some beverage.
    """
    

class Catching(Base):
    """
    A task by which an Agent stops a moving object and gains kinematic control over it, usually by grasping.
    """
    

class PickingUp(Base):
    """
    A task in which the Agent uses one or more of its grippers to grasp a usually stationary object.
    """
    

class CausalEventRole(Base):
    """
    A role filled by a description of some action or process that brings about a motion.
    
    As an example, consider the utterance "the tennisman served the ball by hitting it with the racket." In this utterance, the filler of the CausalEventRole is expressed by the "by hitting it with the racket" constituent.
    """
    

class EventAdjacentRole(Base):
    """
    A role classifying a participant in an event.
    
    In linguistics, this is also known as a thematic role.
    """
    

class CausedMotionTheory(Base):
    """
    A schematic theory describing a situation in which an agent performs an action which causes an object to move along a certain path. A popular example being "He sneezed the napkin off the table." (Goldberg 1995)
    
    Let xA, xP be objects filling the agent, patient roles of this schema. Then one can infer that xA movesObject xP.
    """
    

class ImageSchemaTheory(Base):
    """
    A theory that describes an image-schematic relationship between some entities.
    
    Image schemas are considered as fundamental, pre-conceptual, building blocks of cognition. They were introduced to characterize how human beings are believed to organize and make sense of experience.
    
    For SOMA, whereas the focus of executable schematic theories is to describe how an agent might act, image schematic theories focus on descriptions of how objects behave in the absence of, or after, an active intervention from an agent.
    """
    

class PerformerRole(Base):
    """
    A role classifying an Agent responsible for performing an Action.
    
    The entity playing an PerformerRole is endowed with sentience and the capacity to deliberately choose actions in pursuit of goals. This distinguishes Agents from other causes that could bring an event about.
    """
    

class SourcePathGoalTheory(Base):
    """
    A very general image-schema of the Path family, this schema describes movement along a path from a source towards a goal.
    
    Note: in cognitive linguistics literature, the roles of this schema are Source, Path, Goal. However, to avoid overloading some other terminology in SOMA, we renamed Source to Origin and Goal to Destination.
    
    As yet, this schema is not associated to any object property.
    """
    

class Ceiling(Base):
    """
    The upper interior surface of a room.
    """
    

class RoomSurface(Base):
    """
    The surface of a room.
    """
    

class Floor(Base):
    """
    The lower interior surface of a room.
    """
    

class Wall(Base):
    """
    An upright surface of a building or room.
    """
    

class CeramicCooktop(Base):
    """
    A cooktop that uses heated ceramic elements.
    """
    

class ElectricCooktop(Base):
    """
    A cooktop that uses electricity for heating.
    """
    

class Channel(Base):
    """
    A Channel in a Communication Task is the path of travel by a Message, e.g., via WLAN, air (in the case of a Message classifying soundwaves) or a telephone cable.
    """
    

class PathRole(Base):
    """
    A role that classifies the path of a motion.
    """
    

class CommunicationTask(Base):
    """
    A Task in which two or more Agents exchange information. A CommunicationTask classifies only Events that have only Agents and Social objects as participants.
    
    Of course, the means of exchange is Physical, however this concept is to classify events for which we are not interested in the physical substrate, but rather who communicated and what the information content was.
    """
    

class CheckingObjectPresence(Base):
    """
    A task by which an Agent uses its sensors to check for the presence of a specific object and to obtain some other information about it, e.g. pose.
    """
    

class ChemicalProcess(Base):
    """
    A process by which the chemical constituency of an Entity or set of Entities changes.
    
    In some sense any process that results in entities being created or destroyed might trivially fit here, however this concept is intended specifically for Events where the occuring chemical reactions are of importance.
    """
    

class Process(Base):
    """
    This is a placeholder for events that are considered in their evolution, or anyway not strictly dependent on agents, tasks, and plans. 
    See Event class for some thoughts on classifying events. See also 'Transition'.
    """
    

class Choice(Base):
    """
    The output of a, e.g, Selecting Task.
    """
    

class ResultRole(Base):
    """
    A role classifying the object that is the outcome of a creation or modification action or process.
    """
    

class CircularCylinder(Base):
    """
    A cylinder figure with circular cross section.
    """
    

class CylinderShape(Base):
    """
    A solid geometrical figure with straight parallel sides and a circular or oval cross section.
    """
    

class Classifier(Base):
    ...
    

class StatisticalReasoner(Base):
    ...
    

class ClausalObject(Base):
    """
    A clause is a phrase containing a subject and a predicate.
    """
    

class Phrase(Base):
    ...
    

class Clean(Base):
    """
    A cleanliness region with values considered as clean.
    """
    

class CleanlinessRegion(Base):
    """
    Encodes the cleanliness of an object.
    """
    

class Cleaning(Base):
    """
    This task in which an agent restores all the objects to their destined locations, wiping a specific object
    """
    

class ModifyingPhysicalObject(Base):
    """
    Superconcept for tasks that involve affecting some state that an object is in (e.g. where it is located), without creating or destroying the object.
    """
    

class Purification(Base):
    """
    The disposition of an object (the tool) to change the cleanliness of others.
    """
    

class Cleanliness(Base):
    """
    The quality of being clean.
    """
    

class SocialQuality(Base):
    """
    Any aspect of an entity that specifies social characteristics.
    """
    

class ClientServerSpecification(Base):
    """
    An API Secification that describes the well known Client-Server pattern:
    
    The Client-server model is a distributed application structure that partitions tasks or workloads between the providers of a resource or service, called servers, and service requesters, called clients Often clients and servers communicate over a computer network on separate hardware, but both client and server may reside in the same system. A server host runs one or more server programs, which share their resources with clients. A client usually does not share any of its resources, but it requests content or service from a server. Clients, therefore, initiate communication sessions with servers, which await incoming requests. Examples of computer applications that use the client-server model are email, network printing, and the World Wide Web.
    
    Source: https://en.wikipedia.org/wiki/Client%E2%80%93server_model
    """
    

class ClientRole(Base):
    """
    The Client-server model is a distributed application structure that partitions tasks or workloads between the providers of a resource or service, called servers, and service requesters, called clients Often clients and servers communicate over a computer network on separate hardware, but both client and server may reside in the same system. A server host runs one or more server programs, which share their resources with clients. A client usually does not share any of its resources, but it requests content or service from a server. Clients, therefore, initiate communication sessions with servers, which await incoming requests. Examples of computer applications that use the client-server model are email, network printing, and the World Wide Web.
    
    Source: https://en.wikipedia.org/wiki/Client%E2%80%93server_model
    """
    

class ServerRole(Base):
    """
    The Client-server model is a distributed application structure that partitions tasks or workloads between the providers of a resource or service, called servers, and service requesters, called clients Often clients and servers communicate over a computer network on separate hardware, but both client and server may reside in the same system. A server host runs one or more server programs, which share their resources with clients. A client usually does not share any of its resources, but it requests content or service from a server. Clients, therefore, initiate communication sessions with servers, which await incoming requests. Examples of computer applications that use the client-server model are email, network printing, and the World Wide Web.
    
    Source: https://en.wikipedia.org/wiki/Client%E2%80%93server_model
    """
    

class InterfaceComponentRole(Base):
    ...
    

class Closing(Base):
    """
    A task in which an Agent manipulates a container so as to block access to its interior.
    """
    

class Delivering(Base):
    """
    A task in which an Agent brings an item that the Agent already carries to a specified target.
    """
    

class Fetching(Base):
    """
    A task in which an Agent retrieves an object from a particular location, which puts the object under the Agent's control, who can now e.g. transport the object somewhere else; this task may include repositioning the Agent to better reach the object. Note that while in normal linguistic use fetch can mean transport, we use it here to refer only to (a part of) the first stage of transport.
    """
    

class Lifting(Base):
    """
    todo: how to distinguish from e.g. 'pushing from the table'
    """
    

class Opening(Base):
    """
    A task in which an Agent manipulates a container so as to expose its interior.
    """
    

class Pulling(Base):
    """
    A task in which an Agent moves an object in a direction loosely from the object's center of mass towards the contact point between agent and object.
    """
    

class Pushing(Base):
    """
    A task in which an Agent moves an object in a direction loosely from the the contact point between agent and object towards object's center of mass. todo: define subclass 'PushingOver'? Would we expect two distinct contacts with the same surface then?
    """
    

class Squeezing(Base):
    """
    A task in which an Agent applies pressure to an object they have in their grasp.
    """
    

class ClosingDisposition(Base):
    """
    The disposition of an object to connect with some other, so that a region of space becomes fully enclosed by the object and separate from the exterior. 
    
    Often this connection is between parts of the same object.
    
    Because of the existence of a separation between some interior and exterior space regions, objects that can close are also objects that can contain.
    """
    

class Linkage(Base):
    """
    The disposition of an object (the linked object) to establish a connection with others by being linked together.
    """
    

class Clumsiness(Base):
    """
    A description of clumsy behavior.
    """
    

class CoffeeCarafe(Base):
    """
    A carafe which holds, or at least is intended to usually hold, coffee.
    """
    

class CoffeeTable(Base):
    """
    A small and low table which is typically used as a shelf for small items such as cups, and located in front of a sofa.
    """
    

class CognitiveAgent(Base):
    """
    An agent that is capable to act on its own, in contrast to sub-cognitive Agents, that need to have their intentionality bestowed upon some other agent.
    """
    

class SubCognitiveAgent(Base):
    """
    An agent that is not capable to act on its own, i.e., that is reactive. Its intentionality needs to be bestowed upon from some other agent, that it acts for.
    """
    

class CoilCooktop(Base):
    """
    A cooktop that uses heated metal coils.
    """
    

class Collision(Base):
    """
    A contact event of objects bumping into each other such that their movement is affected.
    """
    

class Extrinsic(Base):
    """
    A physical quality that depends on relationships to other objects, such as the color of an object which depends on light conditions in the environment.
    """
    

class ImperativeClause(Base):
    """
    A clause which commands some agent to perform a task or bring about a state of affairs.
    """
    

class CommitedObject(Base):
    """
    An object committed to a bigger whole. After being committed, the object does not exist anymore in its old form.
    """
    

class ConnectedObject(Base):
    """
    An object that is combined with another object.
    """
    

class CommunicationAction(Base):
    """
    An action in which an Agent uses some actuator for communication purposes.
    """
    

class LinguisticObject(Base):
    ...
    

class CommunicationReport(Base):
    """
    A task in which an Agent endeavors to describe truthfully some state of affairs.
    """
    

class Receiver(Base):
    """
    The role played by an Agent in a Communication Task that perceives and interpretes some incoming Message.
    """
    

class Sender(Base):
    """
    The Role played by an Agent in a Communication Task that emits some Information Realization with the purpose of percipience by some Receiver.
    """
    

class CommunicationTopic(Base):
    """
    A role that appears in communication tasks, and indicates what the communication is about.
    
    CommunicationTopic can only classify a Social object that participates in an Action that is classified as (the execution of) a CommunicationTask.
    
    Note that classifies here is used in the plays-role sense. This isn't to say that the social object, ie the information exchanged in the communication, is an instance of the topic. Rather, the topic role refers to what the information is about.
    
    For example, if the topic of a communication is flowers, this does not mean the words themselves are flowers, merely that, in some sense, they are about flowers.
    """
    

class ResourceRole(Base):
    """
    A role classifying objects that are useful or even necessary to sustain the unfolding of an event.
    
    Resources are usually not agentive; a different set of roles classifies the agentive participants in actions. Likewise, typically resources do not play causative or goal roles for the event.
    
    Resources are often consumed by their participation in an event, but this need not always be the case. An instrument and capital are examples of resources that are reusable.
    """
    

class Composing(Base):
    """
    The disposition of an object (the tool) to change the compositional structure of others.
    """
    

class ComputerLanguage(Base):
    """
    A computer language is a formal language used in communication with a computer.
    
    From Wikipedia: https://en.wikipedia.org/wiki/Computer_language
    """
    

class FormalLanguage(Base):
    """
    A Formal Language consists of words whose letters are taken from an alphabet and are well-formed according to a specific set of rules.
    
    From Wikipedia: https://en.wikipedia.org/wiki/Formal_language
    """
    

class ComputerProgram(Base):
    """
    The Program itself (the specific set of instruction in a Programming Language), not the file that it is contained in nor the implemented algorithm!
    """
    

class ProgrammingLanguage(Base):
    """
    Any Programming Language, including both human-readable like Java and non-human-readable languages like binary machine code.
    """
    

class Conclusion(Base):
    """
    An object that is derived from some premise using some inference rules.
    """
    

class CreatedObject(Base):
    """
    An object that is created.
    """
    

class Knowledge(Base):
    ...
    

class ConditionalSuccedence(Base):
    """
    A relation that holds between two OGPTasks that belong to the same OGPWorkflow, and which means that, if a condition is met, the successor task invocation is to be executed after the predecessor task invocation completes.
    
    The condition is a conjunction of CounterfactualBindings. These bindings may be RoleRoleBindings (meaning, test whether the fillers for these Roles/Parameters are the same) or RoleFillerBindings (meaning, test whether the filler of the Role unifies with the candidate Entity).
    
    An empty conjunction of CounterfactualBindings is assumed to be True.
    Note: in RobotWorkflows, Tasks typically may have several possible successors, to be selected based on condition-testing.
    """
    

class Configuration(Base):
    """
    A description of a State. This includes e.g. what acceptable regions for participant objects of the State might look like.
    A configuration of the world is construed to be stable on its own. Outside disturbances may cause state transitions, and the settling into some other, self-stable configuration.
    
    Several other Description subclasses may function as Configurations. For example, a Goal is a description of a desired State. A Norm describes a State that should be maintained. A Diagnosis describes a State that causes certain symptoms etc.
    
    Note a couple of issues here. First one relates to what "configuration" means; in particular, this doesn't mean a configuration that is unchanging according to any viewpoint. The analogy here is the "macrostate" from thermodynamics: a macrostate with two gases mixed does not mean all gas particles are motionless, but rather that the locations and movements of gas particles are such that any particle is likely to have as many neighbors of one type as the other.
    
    The second issue relates to what is "outside". The state is a configuration of some, but not necessarily all, Entities in the world. Entities not in this configuration are outside of it.
    """
    

class Connectivity(Base):
    """
    The disposition of an object (the connected object) to establish a connection with others.
    """
    

class ContactState(Base):
    """
    Classifies States in which some objects are in contact.
    """
    

class Container(Base):
    """
    A role classifying an object used to contain others.
    """
    

class Containment(Base):
    """
    Classifies States in which an object is placed inside another.
    The disposition of an object (the container) to contain others.
    """
    

class IncludedObject(Base):
    """
    An object that is contained in something. This is meant very general and includes, e.g., elements contained in a set, or things that are spatially contained within the boundaries of some object.
    """
    

class ContainmentState(Base):
    """
    Classifies States in which an object is kept inside another object.
    """
    

class FunctionalControl(Base):
    """
    Classifies States in which an object restricts the movement of another, at least partially. Usually neither object is construed to be an agent.
    
    Note that this State focuses on how the interaction of, usually non-agentive, objects restricts their motion. This is in contrast to Blockage/Accessibility states where the placement of some objects influences the access to some of them by a third, usually agentive party.
    """
    

class ContainmentTheory(Base):
    """
    A schematic theory that describes a functional relation which ensures that the location of some entity, the locatum, is constrained to be within some region which is the interior of some other entity, the relatum.
    
    This is also known as FunctionalControlInternal in GUM-4-space (Bateman et al. 2010).
    
    Let xL, xR be objects filling the locatum, relatum roles of this schema. Then one can infer that xL isInsideOf xR.
    """
    

class ControlTheory(Base):
    """
    A description of functional-spatial configurations where one object controls another object's position in space, e.g. if a pot moves, then the popcorn contained therein moves, as well. Note that the objects do not need to be in direct contact.
    
    Adopted from GUM-4-space (Bateman et al. 2010).
    """
    

class ContinuousJoint(Base):
    """
    A continuous hinge joint that rotates around an axis and has no upper and lower limits.
    """
    

class HingeJoint(Base):
    """
    A joint that rotates along an axis.
    """
    

class FunctionalSpatialSchemaTheory(Base):
    """
    The superclass for theories describing functional spatial relations.
    
    Adopted from GUM-4-space (Bateman et al. 2010).
    """
    

class Cooktop(Base):
    """
    A typically circular surface where food is cooked.
    """
    

class Countertop(Base):
    """
    A flat surface for working on.
    """
    

class Cover(Base):
    """
    An object used to cover up others, such as a lid used as a cover for a pot.
    """
    

class Coverage(Base):
    """
    The disposition of an object (the cover) to hide or to protect objects by covering them. An example is a door that covers items in a container to e.g. prevent dust getting inside of the container.
    """
    

class CoveredObject(Base):
    """
    An object that is covered.
    """
    

class CoverageTheory(Base):
    """
    A schematic theory of a functional relation between two objects such that one of them, the locatum, blocks access to the interior of the relatum.
    
    Let xL, xR be objects filling the locatum, relatum roles of this schema. Then one can infer that xL coversObject xR.
    """
    

class CoveringTheory(Base):
    """
    A schematic theory of how an agent can use an instrument to prevent access to the interior of a patient.
    """
    

class ExecutableSchematicTheory(Base):
    """
    Also known as "executing schemas" or "x-schemas", these were defined by Bergen and Chang in their work "Embodied Construction Grammar in Simulation-Based Language Understanding" as:
    
    "Executing schemas, or x-schemas, are dynamic representations motivated in part by motor and perceptual systems (Bailey 1997; Narayanan 1997), on the assumption that the same underlying representations used for executing and perceiving an action are brought to bear in understanding language about that action. The x-schema formalism is an extension of Petri nets (Murata 1989) that can model sequential, concurrent, and
    asynchronous events"
    
    SOMA does not restrict the formalism of ExecutableSchematicTheories; i.e. they don't have to be Petri Nets.
    
    They maintain their role however as representations able to drive a simulation, at some level of abstraction, of an embodied action. This level of abstraction may be still fairly underspecified as in the case of the original x-schemas and as such not a plan that an agent could run in an actual physical environment without further information.
    """
    

class CrackingTheory(Base):
    """
    A schematic theory of how an agent can inflict damage to the surface of an object.
    """
    

class Creation(Base):
    """
    A process by which an Entity is created in the physical world.
    
    Note, most of the physical Entities we will be considering here are in fact arrangements of many other, smaller physical Entities. Therefore another way to look at this is, a 'Physical creation' is the process by which a set of physical Entities is arranged in a certain way, and the arrangement is then itself considered a physical Entity.
    """
    

class Insertion(Base):
    """
    The disposition of an object (the container) to contain other objects that can be inserted into the container through a portal.
    """
    

class Cuttability(Base):
    """
    The disposition of an object (the barrier) which makes it able to be cut down (into pieces) usually by some other object with role as a Cutter. Such as a cucumber has cuttability disposition which can be cut by a knife.
    """
    

class Tool(Base):
    """
    A role to classify an object used to modify or actuate others.
    """
    

class Cutting(Base):
    """
    The goal of this task is to separate one or more pieces from some target object by means of cutting into its constituent material. Unlike a disassembly, a cut is usually not easily reversible.
    """
    

class Shaping(Base):
    """
    The disposition of an object (the tool) to change the shape of others.
    """
    

class Database(Base):
    """
    A Database is a Software that organizes a collection of data stored and and allows for access, usually via some query engine.
    
    Source: https://en.wikipedia.org/wiki/Database
    """
    

class SoftwareRole(Base):
    """
    A Software Role is a Role applying to only Software and encoding the purpose of the Software: Its Role within Interface Patterns (e.g. Client vs. Server), its functionality (e.g. Database vs. Comnputer Game), and so on.
    """
    

class Deciding(Base):
    ...
    

class DerivingInformation(Base):
    ...
    

class DeductiveReasoning(Base):
    """
    A task in which the Agent, using general logical principles that it considers logically valid, applied to premises that it considers logically true, arrives at conclusions that it considers logically certain.
    
    Deduction is often explained as starting from the "general" (some property X is known about all members of a set S), applying it to the "specific" (some Entity Y is known to belong to set S), to arrive at a specialization of the general property (X applies to Y).
    """
    

class Deformation(Base):
    """
    A process by which a physical Entity changes its shape under the influence of some factors outside of the Entity.
    
    Note, changes of shape may be self-caused; for example, a gas will naturally disperse. This however would be a different type of process (Dispersion).
    
    A soft slab of clay deforming under its own weight on Earth would still count as deformation: it is the gravity of the Earth (so, a factor outside the slab of clay) which makes the slab change shape.
    """
    

class ShapedObject(Base):
    """
    An object undergoing shape change.
    """
    

class FluidFlow(Base):
    """
    A process by which a fluid moves or is moved from a location to another, but such that it maintains its constitution. A fluid is an Entity made of many smaller Entities, loosely bound to each other.
    
    An issue to highlight here is the maintenance of constitution. Fluids-- gases in particular-- are prone to disperse. Such a process is not flow however, because the loose bounds between the constituents become even looser, to the point of the original Entity becoming entirely discombobulated. 
    """
    

class Shifting(Base):
    """
    The disposition of an object (the tool) to change the localization of others.
    """
    

class DependentPlace(Base):
    """
    A feature that is not part of its host, like a hole in a piece of cheese.
    """
    

class Deposit(Base):
    """
    A role classifying an object ontop which others are put to e.g. store them, or to place them in a meaningful way for future activities.
    """
    

class DepositedObject(Base):
    """
    An object placed ontop of another one.
    """
    

class InformationAcquisition(Base):
    """
    A mental task in which the executing agent acquires some information that was not immediately available to it before.
    A synonym might be "Thinking".
    
    Examples include recalling knowledge or inferring some information from other information.
    """
    

class Premise(Base):
    """
    The role of an object that is used to infer some conclusion via some inference rules.
    """
    

class FunctionalPart(Base):
    """
    Parts of an agent or an artifact are considered as functional parts.
    """
    

class Graspability(Base):
    """
    The disposition of an object (e.g. the handle) to afford grasping the object.
    """
    

class DesignedSpade(Base):
    """
    A sharp-edged metal blade used for digging or cutting.
    """
    

class DessertFork(Base):
    """
    A smallar fork for eating desserts e.g. cake.
    """
    

class Destination(Base):
    """
    A role classifying the location where an event or object is directed towards.
    """
    

class DestroyedObject(Base):
    """
    An object that is detroyed.
    """
    

class Destruction(Base):
    """
    A process by which a physical Entity is destroyed.
    
    Note, most of the physical Entities we are concerned with are actually arrangements of many smaller physical Entities, so another way to look at this is that a 'Physical destruction' is a process by which an arrangement of physical Entities, which was previously itself considered a physical Entity, is changed to such an extent that it is no longer recognized as continuing to exist.
    """
    

class DetectedObject(Base):
    """
    An object that is detected.
    """
    

class DeviceState(Base):
    """
    A quality belonging to a device which indicates its overall functional state.
    """
    

class DeviceStateRange(Base):
    """
    This class defines the values that a device state can take.
    """
    

class DeviceTurnedOff(Base):
    """
    A value indicating a device is not in operation.
    """
    

class DeviceTurnedOn(Base):
    """
    A value indicating a device is in operation.
    """
    

class DexterityDiagnosis(Base):
    """
    A description of the dexterity of a system, possibly in comparison to another system.
    """
    

class Dicing(Base):
    """
    A particular kind of cutting where the goal is to produce small pieces out of some object or material. Unlike slices, the pieces to be obtained do not have one or two dimensions being more prominent than others. "Dice", the pieces dicing results in, are approximately cubic.
    """
    

class InformationRealization(Base):
    """
    A concrete realization of an InformationObject, e.g. the written document (object) containing the text of a law, a poetry reading (event), the dark timbre (quality) of a sound (event) in the execution (event) of a musical composition, realizing a 'misterioso' tempo indication.
    
    The realization of an information object also realizes information about itself. This is a special semiotic feature, which allows to avoid a traditonal paradox, by which an information is often supposed to be about itself besides other entities (e.g. the information object 'carpe diem' is about its meaning in Horace's Odes (let alone its fortune in Western culture and beyond), but also about its expression in context: 'dum loquimur, fugerit invida aetas: carpe diem, quam minimum credula postero', with the sound and emotional relations that it could activate.
    This is expressed in OWL2 with a local reflexivity axiom of the dul:InformationRealization class.
    """
    

class DirectedMotion(Base):
    """
    A Motion that is considered to be toward a location or along a path. It is not important that the final location or path be the intention of some Agent, but it is considered that the important feature of this Motion is that it has a path and/or destination.
    """
    

class UndirectedMotion(Base):
    """
    A Motion of a physical Entity for which a destination or path are unknown and/or considered irrelevant; the important aspect about this Motion is simply that it occurs, rather than where it is headed or how it proceeds towards it.
    """
    

class Dirty(Base):
    """
    A cleanliness region with values considered as dirty.
    """
    

class Discourse(Base):
    """
    A mental task, in which two or more agents discuss some topic via multiple Illocutionary acts, which are part of this task.
    """
    

class Dispenser(Base):
    """
    A small container used to store and serve some flavoring substance for a dish or beverage.
    """
    

class Distancing(Base):
    """
    A task in which an Agent increases its distance from some location.
    """
    

class Dreaming(Base):
    """
    Any form of re-processing episodic memories for long-term memory by natural or aritifical agents.
    """
    

class Driving(Base):
    """
    A process type classifying a motion of a body that exists because this body is attached to and controls some other moving body, usually a vehicle.
    """
    

class Flying(Base):
    """
    A process type classifying a motion of a body that, through its own power, keeps itself aloft.
    """
    

class Swimming(Base):
    """
    A motion of some body through water. The body provides the power for the motion.
    """
    

class Walking(Base):
    """
    An agent, under its own power, moves over some solid surface.
    """
    

class Dropping(Base):
    """
    The dropped object falls mainly under the influence of gravity. However, an agent may also drop something during navigation. The difference to 'Throwing' is that there is no 'Limb motion' which is a constitiuent of the action.
    
    'Dropping' is intentional. Dropping by accident may not has a phase to release the grasp. It could be that the grasp was not strong enough and the objects "slips" away.
    """
    

class Placing(Base):
    """
    Distinguished from Positioning in that this task is more about placing an object at a functionally specified location (e.g., place the cup on the table) as opposed to positioning an object at a location defined by coordinates or a region of coordinates (position the cup at xyz).
    """
    

class ESTSchemaTheory(Base):
    """
    A schematic theory that describes the existence of an entity.
    
    Developmental psychology posits that "object permanence" (the assumption that physical objects keep existing even when the agent doesn't perceive them, which consequently informs reasoning about where an object should be, even when perception of it is lost) is a cognitive ability that is not, at least in the very young human child, immediately manifest. Rather, it seems learned via interaction, and usually is among an infant's cognitive repertoire by age 2.
    
    In SOMA, we represent this ability of a cognitive agent as an ability to generate and maintain ESTSchemaTheories. Each instance of such a theory refers to one particular physical object, the one that the instance of the ESTSchemaTheory asserts to exist.
    
    Because each instance of an ESTSchemaTheory refers to a single object, ESTSchemaTheories are not associated to any relation between OWL individuals.
    """
    

class ExistingObjectRole(Base):
    """
    A role that requires of its filler simply to exist, unlike other roles that may demand e.g. agentive or instrumental participation in some executable schema or plan (AgentRole and Instrument respectively).
    
    The intention behind such a simple role is to have a way to represent, in a schematic formalism used to describe some scene, that an object is present. In particular, the schema used to convey this information is the ESTSchemaTheory, which has ExistingObjectRole as its sole defined role.
    """
    

class Effort(Base):
    """
    A parameter describing the amount of force to be exerted by some actuator.
    """
    

class EnclosedObject(Base):
    """
    An object included within the spatial boundaries of another object.
    """
    

class Enclosing(Base):
    """
    The disposition of an object (the container) to contain other objects by enclosing them to prevent their free movement.
    """
    

class EndEffectorPositioning(Base):
    """
    A task in which an Agent places its end effectors at particular poses.
    """
    

class Manipulating(Base):
    """
    Tasks where the goal is to move the prehensile effectors, ie. hands, of an agent so as to achieve some spatial or functional relation with some manipulated object. 
    
    Spatial relations refer to positioning the hands in certain ways relative to the manipulated object, for example nearing or distancing them, or aligning them with some relevant axis.
    
    Functional relations here refer to interactions between the hands and manipulated object which constrain the possible behavior of the object. Examples of functional relations in manipulation are support and kinematic control.
    
    Note that manipulation tasks are usually performed with the intention of moving an object in some way, so there is a large conceptual overlap between Manipulating and Actuating.
    
    However these concepts are nonetheless distinguished in what they "profile", ie. what they focus on as particularly important.
    
    Actuating profiles the movement of the object itself.
    
    Manipulating profiles the movement of the hands and the functional relations, such as kinematic control, they establish with the manipulated object.
    
    Note: we employ Manipulating here in its literal, original sense, of using hands for some purpose, and not in the metaphorical sense of exerting psychological pressure on someone.
    """
    

class Episode(Base):
    """
    
    """
    

class ExcludedObject(Base):
    """
    An object that is not contained in something. This is meant very general and includes, e.g., elements excluded from a set, or things that are spatially excluded from the boundaries of some object.
    """
    

class ExecutableFile(Base):
    """
    An Executable File, sometimes simply referred to as an executable or binary, causes a computer "to perform indicated tasks according to encoded instructions", as opposed to a data file that must be interpreted (parsed) by a program to be meaningful.
    
    The exact interpretation depends upon the use. "Instructions" is traditionally taken to mean machine code instructions for a physical CPU. In some contexts, a file containing scripting instructions (such as bytecode) may also be considered executable.
    """
    

class ExecutableCode(Base):
    """
    Executable Code is Code that when compiled / interpreted, has some clear entrance point and can be executed. Note the difference to an Executable File, which is the file that contains such (compiled) code.
    """
    

class ExecutableFormat(Base):
    """
    An Executable Format is a File Format that allows computers to directly execute the content. Examples are MZ (DOS) or COFF.
    """
    

class SchematicTheory(Base):
    """
    A theory used to describe, analyze, and reason with the meaning of a linguistic message.
    
    Note that such theories are useful both when analyzing an actual linguistic production, and when creating a linguistic production to describe some observed experience.
    """
    

class ExecutableSoftware(Base):
    """
    Executable Software is Software that can directly be executed, in comparison to a Software Library, which might only contain functionality via interfaces, but offers no execution entry point.
    """
    

class Software(Base):
    ...
    

class RelationAdjacentRole(Base):
    """
    A role classifying an object participating in some relation, e.g. a participant in a spatial relation or a linguistic fragment in a rhetorical relation to another.
    """
    

class ExperiencerRole(Base):
    """
    A role used in frame semantics to classify agents performing perception actions, or being the subjects affected by some biological process.
    """
    

class ExtractedObject(Base):
    """
    An object that is removed from a container or system.
    """
    

class PhysicalQuality(Base):
    """
    Any aspect of an entity that is dependent on its physical manifestation.
    """
    

class FailedAttempt(Base):
    """
    A description of a failed attempt to achieve some goal.
    """
    

class FaultySoftware(Base):
    """
    A description of a situation where some software has a bug.
    """
    

class SoftwareDiagnosis(Base):
    """
    A diagnosis of the software of a system.
    """
    

class PhysicalAcquiring(Base):
    """
    The goal of this task is to make some object usable for other tasks, by possibly changing its physical state. Usually, it overlaps some task that describes the manner in which an object is obtained.
    
    The prototypical example of PhysicalAcquiring is picking up an object.
    
    Note that buying an object is NOT PhysicalAcquiring. Buying, or ownership transfer in general, also involves an adjustment in social structures describing ownership.
    """
    

class Configuration(Base):
    """
    A collection whose members are 'unified', i.e. organized according to a certain schema that can be represented by a Description.
    Typically, a configuration is the collection that emerges out of a composed entity: an industrial artifact, a plan, a discourse, etc.  
    E.g. a physical book has a configuration provided by the part-whole schema that holds together its cover, pages, ink. That schema, based on the individual relations between the book and its parts, can be represented in a reified way by means of a (structural) description, which is said to 'unify' the book configuration.
    """
    

class Finger(Base):
    """
    A limb used for grasping objects.
    """
    

class Hand(Base):
    """
    A prehensile effector including palm, fingers, and thumb.
    """
    

class FixedJoint(Base):
    """
    A joint that cannot move, designed to fixiate links.
    """
    

class MovableJoint(Base):
    """
    A joint where the two connected links can move relative to each other in some dimension.
    """
    

class Flipping(Base):
    """
    The task in which the agent turns an object over by using a tool or by manipulating
    """
    

class FloatingJoint(Base):
    """
    A joint that allows motion for all 6 degrees of freedom.
    """
    

class MovedObject(Base):
    """
    An object undergoing location change.
    """
    

class Focusing(Base):
    """
    The mental task to center the attention to some subject.
    """
    

class Foolishness(Base):
    """
    A description of foolish behavior.
    """
    

class ForgettingIncorrectInformation(Base):
    """
    A mental task in which the executing agent aims to correct its present information by deleting an incorrect information.
    """
    

class InformationDismissal(Base):
    """
    A mental task in which the executing agent dismisses some information.
    
    An example is forgetting some knowledge.
    """
    

class ForgettingIrrelevantInformation(Base):
    """
    A mental task in which the executing agent aims to clean up its present information by deleting an irrelevant information.
    """
    

class Language(Base):
    """
    A Language is a structured System for communication.
    
    From Wikipedia: https://en.wikipedia.org/wiki/Language
    """
    

class FreezerCompartment(Base):
    """
    A freezer compartment used to regulate the temperature of contained objects to a value below zero degree Celsius.
    """
    

class Item(Base):
    """
    A role played by a non-agentive object operated on by an action.
    """
    

class FunctionalDesign(Base):
    """
    The design of an object from functionality point of view. A functional design is useful to develop complex modular objects with components that have a specific purpose, and can function with minimum side effect on other components of that object. 
    """
    

class FunctionalDiagnosis(Base):
    """
    An internal diagnosis of a system.
    """
    

class PhysicalBody(Base):
    """
    Physical bodies are PhysicalObject(s), for which we tend to neutralize any possible artifactual character. They can have several granularity levels: geological, chemical, physical, biological, etc.
    """
    

class LocatumRole(Base):
    """
    Denotes the object with primary focal prominence in a spatial or spatio-temporal schema. Terminological variants that appear in the literature on cognitive linguistics include Figure (Talmy 1983) and Trajector (Langacker 1986).
    """
    

class RelatumRole(Base):
    """
    Denotes the reference object in a spatial or spatio-temporal schema, i.e. the object with secondary focal prominence. Terminological variants: Ground (Talmy 1983), Landmark (Langacker 1986).
    """
    

class GasCooktop(Base):
    """
    A cooktop that uses burning gas for heating.
    """
    

class GetTaskParameter(Base):
    """
    A task in which an Agent computes some parameter relevant for another task.
    """
    

class Planning(Base):
    """
    A Mental task in which the Agent endeavours to create a sequence of actions for itself which, if followed, will bring about a particular state of affairs in the world. This particular state of affairs is known to the agent and is often called the goal state of the planning action. Planning commits itself to feasibility: the Agent attempts to find a sequence of actions that it believes it will actually be able to perform.
    """
    

class GraphDatabase(Base):
    """
    A Graph Database is a Database that uses graph structures for semantic queries with nodes, edges, and properties to represent and store data.
    """
    

class GraphQueryLanguage(Base):
    """
    A Query Language that is designed for communication with some Graph Database.
    """
    

class QueryLanguage(Base):
    """
    Query languages, are Computer languages used to make queries in databases and information systems (Source: https://en.wikipedia.org/wiki/Query_language).
    
    Note that despite their name, Query languages typically come with syntax and semantic to not only ask for information, but also provide them, e.g., via SQL Update. In that sense, the term "query" from above refers to any formal object of information exchange with a database.
    """
    

class GraspTransfer(Base):
    """
    A task in which an Agent switches which of its end effectors holds an object.
    """
    

class Grasping(Base):
    """
    A task in which an Agent uses its end effectors to grasp an object, thus gaining kinematic control over it.
    """
    

class Releasing(Base):
    """
    A task in which an agent relinquishes its kinematic control over an object, typically by releasing it from its grasp.
    """
    

class GraspingMotion(Base):
    """
    A process type classifying a motion by which some end effector acquires kinematic control over some other object.
    """
    

class IntermediateGrasp(Base):
    """
    A kind of grasp that acquires kinematic control over the gripped object, but without attempting to achieve either strong holding force nor precision of subsequent movement of the gripped object.
    """
    

class PowerGrasp(Base):
    """
    An Agent grasps an object, and focuses on obtaining a strong grasping force upon it, resulting in a grasp able to resist significant outside disturbances. This is useful when using tools with which to later exert force on other things, e.g. when hammering nails.
    """
    

class PrecisionGrasp(Base):
    """
    An Agent grasps an object, and focuses on obtaining precise kinematic control over it. This is useful for then following precise movements, e.g. when writing.
    """
    

class PrehensileMotion(Base):
    """
    A motion of a prehensile effector.
    """
    

class ReleasingMotion(Base):
    """
    A motion by which a prehensile effector relinquishes kinematic control over an object.
    """
    

class GreenColor(Base):
    """
    A color region with dominant green color.
    """
    

class Gripper(Base):
    """
    A mechanical device that grasps and holds things.
    """
    

class PrehensileEffector(Base):
    """
    An effector used to grasp objects, such as a hand of a human, or the long prehensile tail of a monkey.
    """
    

class HardwareDiagnosis(Base):
    """
    A diagnosis of the hardware of a system.
    """
    

class HasQualityRegion(Base):
    """
    The relation between an individual quality and a region.
    todo(DB): added for NEEMs (quale change), but not sure yet about it...
    """
    

class Head(Base):
    """
    A functional part of the body responsible for carrying high bandwidth sensors, i.e., camera.
    """
    

class HeadMovement(Base):
    """
    The Agent moves a part of their body carrying high-bandwidth sensors such as cameras. The movement of other body parts is not significant.
    todo: (Mihai:) This seems too specific, given the "Directed"/"Undirected motion" categories.
    """
    

class HeadTurning(Base):
    """
    A process type classifying a motion of an Agent's head such that the direction this head faces changes relative to the facing direction of the Agent's body as a whole.
    """
    

class Holding(Base):
    """
    A task by which an Agent keeps an object over which it has kinematic control, typically via grasping, at some specified pose.
    """
    

class HostRole(Base):
    """
    In the Plug-in-Host pattern, a Host application provides services which the Plug-in can use, including a way for Plug-ins to register themselves with the Host application and a protocol for the exchange of data withPplug-ins. Plug-ins depend on the services provided by the host application and do not usually work by themselves. Conversely, the host application operates independently of the plug-ins, making it possible for end-users to add and update plug-ins dynamically without needing to make changes to the host application.
    
    Source: https://en.wikipedia.org/wiki/Plug-in_(computing)
    """
    

class PluginSpecification(Base):
    """
    The Specification of a Plugin interface defines how a Host and a Plug-in function together and exchange information.
    """
    

class Hotplate(Base):
    """
    A flat heated surface used for cooking food or keeping it hot.
    """
    

class HumanreadableProgrammingLanguage(Base):
    """
    A Programming language like Java, Python etc. but not binary machine code.
    """
    

class SourceCode(Base):
    """
    The Source Code itself (the specific set of instruction in a human-readable Programming Language), not the file that it is contained in nor the implemented algorithm!
    """
    

class HumanActivityRecording(Base):
    """
    An episode in which one or more human beings perform an activity and are recorded doing so.
    """
    

class Imagining(Base):
    """
    A Mental task in which the Agent constructs a mental representation of a possible world. An Agent performing an Imagining activity does not aim to construct a representation that aspires to be faithful to some past, present, or future state of affairs of the actual world it is embodied in.
    """
    

class Impediment(Base):
    """
    The disposition of an object (the obstacle) to prohibit certain ways of entering or leaving a space or group. An example is a doorstopper constraining a door, prohibiting it to enter the area behind it.
    """
    

class Obstacle(Base):
    """
    An object used to restrict access to a protected space or group.
    """
    

class RestrictedObject(Base):
    """
    An object with restrictions to access something.
    """
    

class StateTransition(Base):
    """
    A transition between two states brought about by the Action of some Agent.
    """
    

class Inability(Base):
    """
    A description of a situation with a goal that some system is unable to achieve.
    """
    

class IncompatibleSoftware(Base):
    """
    A description of a situation where two software systems are incompatible with each other.
    """
    

class InductionCooktop(Base):
    """
    A type of cooktop where heat is induced in the cookware. Only special induction cookware can be used with this type of cooktop.
    """
    

class InductiveReasoning(Base):
    """
    A task in which the Agent endeavors to accumulate confidence in some general statement about the world, by gathering instances in which this general statement appears to apply. Note that perfect confidence can never be guaranteed by induction.
    
    Induction is often described as a move from many "specifics" (swan A is white, swan B is white, swan C is white, ...) to the "general" (all swans are white).
    """
    

class Infeasibility(Base):
    """
    A description of a situation with a goal that is impossible to achieve in some situational context.
    """
    

class InferenceRules(Base):
    """
    The role of an object that is used to derive a conclusion from some premises.
    """
    

class InformationRetrieval(Base):
    """
    A mental task in which an Agent recalls some knowledge that has been memorized previously.
    
    Examples include a human remembering some information or a computer retrieving knowledge from a database.
    
    The difference to Remembering is that for this Task, we are concerned with knowledge about a previous world state. Memory Retrieval is more general in the sense that it also includes the retrieval of learned facts and rules.
    """
    

class InformationStorage(Base):
    """
    A mental task in which the executing agent persists some information for later recall, if necessary.
    
    An example is learning new knowledge.
    """
    

class StoredObject(Base):
    """
    An object being stored into some other object, usually inside a container.
    """
    

class InsertedObject(Base):
    """
    An object inserted into another object.
    """
    

class Instructions(Base):
    """
    The role of a plan to follow during an execution task.
    """
    

class Interpreting(Base):
    """
    A task in which an Agent interpretes some information, e.g., makes sense of some incoming message or its visible surroundings.
    """
    

class InterrogativeClause(Base):
    """
    A clause which makes a request, typically information, of some agent.
    
    Note that in a semantic sense such clauses always request information, but in a pragmatic sense they can be used to convey commands or requests for action, such as e.g. "can you close the door?" The question is not just a request for information about ability, but a request to perform a task.
    """
    

class Introspecting(Base):
    """
    A mentalk task in which an Agent gathers and processes information about its own mental tasks via, e.g., Meta Reasoning.
    """
    

class JamJar(Base):
    """
    A jar which holds, or at least is intended to usually hold, jam.
    """
    

class KineticFrictionAttribute(Base):
    """
    Friction that occurs when two touching objects are moving relative to each other.
    """
    

class KinoDynamicData(Base):
    """
    An InformationObject containing data about how a physical object is put together such that its parts may move relative to each other, and what the physical characteristics of those parts are.
    """
    

class KitchenUnit(Base):
    """
    A piece of a fitted furniture typically containing cupboards, a sink, and an oven.
    """
    

class KnowledgeRepresentationLanguage(Base):
    """
    A Knowledge Representation Language is a Language with fixed semantics and syntax to describe some knowledge. Examples are JSON and the different OWL Profiles.
    """
    

class Labeling(Base):
    ...
    

class Text(Base):
    """
    Any Information Object defined using a Language that, when realized through a symbolic writing system, can be read and made sense of.
    
    One may define Text as anything that can be made sense of, including e.g., Speech or Paintings.
    
    However, the argument can be made that Speech or Paintings are not considered Text as they cannot be EXACTLY realized by a symbolic writing system: Speech may loose punctuation, Paintings their original appearance.
    On the other hand, this might not be true as both could be encoded in a binary format that can be interpreted using a language (eg., mp3, png).
    """
    

class Leaning(Base):
    """
    An Agent pitches its body in some direction.
    """
    

class PosturalMoving(Base):
    """
    The Agent changes or takes an overall configuration of its body but is otherwise not significantly affecting other objects nor moving a significant amount from its original location.
    
    Posture changes may take place as part of other actions, for example turning when walking.
    """
    

class Learning(Base):
    """
    The mental task of storing information for later use.
    
    This is more general than memorizing, as the later only captures declarative knowledge.
    """
    

class Leg(Base):
    """
    A limb on which an agent walks or stands.
    """
    

class LimbMotion(Base):
    """
    An Agent moves its limbs in some way.
    """
    

class LinkedObject(Base):
    """
    An object that is linked to some other object.
    """
    

class LinkageState(Base):
    """
    Classifies States in which two objects are in a rigid connection, such that the movement of one determines the movement of the other.
    """
    

class SpatioTemporalRole(Base):
    """
    Roles that classify entities which locate an event or object in space and time.
    """
    

class SpatialRelationRole(Base):
    """
    Roles classifying entities participating in some spatial relation.
    """
    

class LocutionaryAction(Base):
    """
    A Locutionary Act is the performance of an utterance (source: https://en.wikipedia.org/wiki/Locutionary_act).
    
    We additionally require a Locutionary Act to be performed by an Agent, not an Actor - this is what sets it apart from a Communication Action.
    """
    

class LookingAt(Base):
    """
    better: Gazing
    """
    

class LookingFor(Base):
    """
    A task by which an Agent uses its perception apparatus to check for the presence of an object in some specified area.
    """
    

class Lowering(Base):
    """
    A task in which an Agent reduces the elevation at which they hold an item.
    """
    

class PhysicalAction(Base):
    """
    An action performed by an agent by using its body in some way to interact with the physical world, e.g., through manipulation of objects, or by changing the posture.
    """
    

class MarkupLanguage(Base):
    """
    Markup refers to data included in an electronic document which is distinct from the document's content in that it is typically not included in representations of the document for end users, for example on paper or a computer screen, or in an audio stream. Markup is often used to control the display of the document or to enrich its content to facilitate automated processing. A markup language is a set of rules governing what markup information may be included in a document and how it is combined with the content of the document in a way to facilitate use by humans and computer programs.
    
    From Wikipedia: https://en.wikipedia.org/wiki/Markup_language
    """
    

class Masterful(Base):
    """
    A description of masterful behavior.
    """
    

class Material(Base):
    """
    The matter from which a thing is made.
    """
    

class MedicalDiagnosis(Base):
    """
    A functional diagnosis of an organism.
    """
    

class Memorizing(Base):
    """
    An atomic mental task in which an Agent saves some (declarative) information for later retrieval.
    
    Examples include a student learning vocabularies or a computer saving some information to a database.
    """
    

class MentalAction(Base):
    """
    An Event construed as the Agent participant affecting Entities that are representations of actual or potential Entities or Events in the physical world in which the Agent is embodied. These representations are maintained by the Agent participant in the 'Mental action' event.
    
    One could argue Mental actions are all Physical actions, because anything the Agent may use to maintain such representations will be physical things, However, we make this distinction because for Mental actions it is less important to consider the physical support of the representation and how it changes, and more important to track how the information content of the representation changes.
    """
    

class MeshShape(Base):
    """
    A solid geometrical figure described in a mesh file.
    """
    

class MeshShapeData(Base):
    """
    An InformationObject containing data about the geometry of a physical object.
    """
    

class MetaCognitionEvaluationTopic(Base):
    """
    A topic used while an Agent describes its own cognitive processes and acions to evaluate them according to some metric.
    """
    

class MetaCognitionTopic(Base):
    """
    A topic for a description that an Agent might make of its own cognitive processes and actions.
    """
    

class MetaCognitionMemoryTopic(Base):
    """
    A topic used while an Agent describes its own cognitive processes and actions, and which covers descriptions of what memories are involved in them.
    """
    

class MetaCognitionPlanningTopic(Base):
    """
    A topic used while an Agent describes the planning it does for its own cognitive processes and actions.
    """
    

class ThinkAloudTopic(Base):
    """
    A topic relevant for a think-aloud communication.
    """
    

class MetacognitiveControlling(Base):
    """
    The concious or subconcious task to control the own mental processes, e.g., evaluating them and instructing the own mind to shift attention.
    """
    

class MetacognitiveMonitoring(Base):
    """
    The task to label the processes and states of the own mind, e.g., to interprete the feeling of knowing an information but not being able to retrieve it at the moment as a tip-of-the-tongue event.
    """
    

class Mixing(Base):
    """
    A task by which an Agent combines several entities, such that the combination is difficult or in practice impossible to reverse.
    """
    

class MixingTheory(Base):
    """
    A schematic theory about how an agent can mix a fluid or particulate object.
    """
    

class MonitoringJointState(Base):
    """
    A task in which the Agent keeps track of the physical state of its joints, e.g. their positions, velocities, efforts.
    """
    

class Proprioceiving(Base):
    """
    A task in which the Agent gathers and interprets sensor information about itself.
    """
    

class ProcessFlow(Base):
    """
    A description that structures a Process.
    """
    

class PhysicsProcess(Base):
    """
    A process involving physical objects and phenomena which does not change the chemical constituency of the affected objects.
    """
    

class MovingAway(Base):
    """
    A process type classifying a motion by which some Agent puts distance between itself and another object or location.
    """
    

class MovingTo(Base):
    """
    A task in which an Agent moves towards a location.
    """
    

class NaturalLanguage(Base):
    """
    A Natural Language is any language that has evolved naturally in humans through use and repetition without conscious planning or premeditation.
    
    From Wikipedia: https://en.wikipedia.org/wiki/Natural_language
    """
    

class NaturalLanguageText(Base):
    """
    A Text in a Natural Language.
    """
    

class NutellaJar(Base):
    """
    A jar which holds, or at least is intended to usually hold, Nutella.
    """
    

class Ontology(Base):
    """
    An ontology encompasses a representation, formal naming, and definition of the categories, properties, and relations between the concepts, data, and entities that substantiate one, many, or all domains of discourse. More simply, an ontology is a way of showing the properties of a subject area and how they are related, by defining a set of concepts and categories that represent the subject.
    
    From Wikipedia: https://en.wikipedia.org/wiki/Ontology_(information_science)
    """
    

class OntologyLanguage(Base):
    """
    An Ontology Language is a Knowledge Representation Language to describe knowledge about properties of a subject area and how they are related, by defining a set of concepts and categories that represent the subject using logic. Examples are the different OWL Profiles.
    
    Source: https://en.wikipedia.org/wiki/Ontology_(information_science)
    """
    

class Option(Base):
    """
    The Role of objects that are used, e.g., in Selecting Tasks.
    """
    

class FormalEntity(Base):
    """
    Entities that are formally defined and are considered independent from the social context in which they are used. They cannot be localized in space or time. Also called 'Platonic entities'.
    Mathematical and logical entities are included in this class: sets, categories, tuples, costants, variables, etc.
    Abstract formal entities are distinguished from information objects, which are supposed to be part of a social context, and are localized in space and time, therefore being (social) objects.
    For example, the class 'Quark' is an abstract formal entity from the purely set-theoretical perspective, but it is an InformationObject from the viewpoint of ontology design, when e.g. implemented in a logical language like OWL.
    Abstract formal entities are also distinguished from Concept(s), Collection(s), and Description(s), which are part of a social context, therefore being SocialObject(s) as well.
    For example, the class 'Quark' is an abstract FormalEntity from the purely set-theoretical perspective, but it is a Concept within history of science and cultural dynamics.
    
    These distinctions allow to represent two different notions of 'semantics': the first one is abstract and formal ('formal semantics'), and formallyInterprets symbols that are about entities whatsoever; for example, the term 'Quark' isAbout the Collection of all quarks, and that Collection isFormalGroundingFor the abstract class 'Quark' (in the extensional sense). 
    The second notion is social, localized in space-time ('social semantics'), and can be used to interpret entities in the intensional sense. For example, the Collection of all quarks isCoveredBy the Concept 'Quark', which is also expressed by the term 'Quark'.
    """
    

class Singleton(Base):
    """
    A 'Set' that contains exactly one member.
    """
    

class Orienting(Base):
    """
    A task in which an Agent adjusts the orientation of an object.
    """
    

class Positioning(Base):
    """
    A task in which an Agent places an object at a particular position.
    """
    

class Origin(Base):
    """
    A role classifying the location where an event or object originated.
    """
    

class Oven(Base):
    """
    An appliance with an enclosed compartment that exposes food to heat
    """
    

class TemperingByHeating(Base):
    """
    A disposition of an object to change others by applying heat energy to them.
    """
    

class ParkingArms(Base):
    """
    A task by which an Agent arranges its arms in such a way so as to minimize opportunities for collision while moving through the environment.
    """
    

class PepperShaker(Base):
    """
    A shaker which holds, or at least is intended to usually hold, pepper.
    """
    

class PhaseTransition(Base):
    """
    An EventType that classifies processes by which matter changes from some distinguishable phase to another. We will use this to refer to the classical phase transitions between Solid, Liquid, Gaseous, Plasma etc. phases.
    """
    

class PhysicalAccessibility(Base):
    """
    Classifies States in which an item is placed in a container or protected by a protector, but the placement of the item and container is such that a, usually agentive, accessor may nevertheless reach the item in order to perform a task.
    """
    

class PhysicalBlockage(Base):
    """
    Classifies States in which an object, in general called restrictor, blocks access to an item. A usually agentive accessor, whose access is blocked, may be specified.
    """
    

class PhysicalExistence(Base):
    """
    A State in which an Entity is said to exist in the physical world.
    """
    

class PhysicalState(Base):
    """
    A State describing how Entities in the physical world relate to each other.
    """
    

class PlacingTheory(Base):
    """
    A schematic theory of how an agent can place a patient at a particular goal location.
    """
    

class PlanarJoint(Base):
    """
    A joint that allows motion in a plane perpendicular to an axis.
    """
    

class PluginRole(Base):
    """
    In the Plug-in-Host pattern, a Host application provides services which the Plug-in can use, including a way for Plug-ins to register themselves with the Host application and a protocol for the exchange of data withPplug-ins. Plug-ins depend on the services provided by the host application and do not usually work by themselves. Conversely, the host application operates independently of the plug-ins, making it possible for end-users to add and update plug-ins dynamically without needing to make changes to the host application.
    
    Source: https://en.wikipedia.org/wiki/Plug-in_(computing)
    """
    

class Pourable(Base):
    """
    The disposition of a fluid or substance which makes it possible to pour it out of a container and into or onto other objects.
    """
    

class PouredObject(Base):
    """
    An object being poured into or onto some other object. A role of some fluid or substance that is the patient of pouring task.
    """
    

class Pouring(Base):
    """
    A task in which an agent lets liquid substance to flow out of an object. The agent has a kinematic control over the object.
    """
    

class PouringInto(Base):
    """
    The task in which the agent pours the substance into another object.
    """
    

class PouringOnto(Base):
    """
    The task in which an agent pours the substance on top of an object
    """
    

class Prediction(Base):
    """
    A Mental task in which the Agent endeavours to construct a representation of a future state of the world. Prediction commits itself to some degree of accuracy: the Agent believes that eventually something similar to the predicted state will come to pass.
    """
    

class Prospecting(Base):
    """
    A Mental task in which an Agent endeavours to construct a representation of a future state of affairs of the world it is embodied in.
    """
    

class Predilection(Base):
    """
    The relation between a 'Preference' and the 'Order' that the 'Preference' defines over Situations.
    
    For the complete model, see 'Preference'.
    """
    

class SocialRelation(Base):
    """
    Any social relationship
    """
    

class PreferenceOrder(Base):
    """
    The relation between a 'Preference' and the 'Order' that the 'Preference' defines over Descriptions of Situations.
    """
    

class PreferenceRegion(Base):
    """
    The 'Region' of 'Preference's, containing all possible 'Order's between all possible 'Situation's.
    """
    

class SocialObjectAttribute(Base):
    """
    Any Region in a dimensional space that is used to represent some characteristic of a SocialObject, e.g. judgment values, social scalars, statistical attributes over a collection of entities, etc.
    """
    

class PrismaticJoint(Base):
    """
    A sliding joint that slides along an axis, and has a limited range specified by the upper and lower limits.
    """
    

class Progression(Base):
    """
    A situation that sattisies a description of how a process evolves over time.
    """
    

class Protector(Base):
    """
    A role classifying an object that protects another by preventing other entities from coming in contact with the protected object.
    """
    

class ProximalTheory(Base):
    """
    An image schematic theory that describes a qualitative spatial relation indicating relative proximity, as expressed by the prepositions 'near', 'close to', 'next to', etc.
    
    It may seem that proximity is a very simple notion, requiring no sophisticated theoretical underpinnings. However, proximity is an extremely object- and purpose-dependent relation. A store next door is in a closeness relation to a person, and so is the Sun in the sky, despite the physical distances being different by several orders of magnitude.
    
    As such, a theory, i.e. a description of what a particular kind of closeness means and in which contexts it applies, is necessary.
    
    Adopted from GUM-4-space (Bateman et al. 2010).
    
    Let xL, xR be objects filling the locatum, relatum roles of this schema. Then one can infer that xL 'near to' xR.
    """
    

class PushingAway(Base):
    """
    A task in which an Agent pushes an object in front of themselves.
    """
    

class PushingDown(Base):
    """
    A task in which an Agent pushes an object downwards.
    """
    

class PuttingDown(Base):
    """
    A task in which an Agent puts down an object they have kinematic control over, e.g. a grasped object.
    """
    

class QualityTransition(Base):
    """
    todo(DB): added for NEEMs (quale change), but not sure yet about it...
    """
    

class Transition(Base):
    """
    A transition is a Situation that creates a context for three TimeInterval(s), two additional different Situation(s), one Event, one Process, and at least one Object: the Event is observed as the cause for the transition, one Situation is the state before the transition, the second Situation is the state after the transition, the Process is the invariance under some different transitions (including the one represented here), in which at least one Object is situated. Finally, the time intervals position the situations and the transitional event in time.
    This class of situations partly encodes the ontology underlying typical engineering algebras for processes, e.g. Petri Nets. 
    A full representation of the transition ontology is outside the expressivity of OWL, because we would need qualified cardinality restrictions,  coreference, property equivalence, and property composition.
    """
    

class Query(Base):
    """
    A role played by some Information Realization that carries meaning, where this meaning is a query of some sort.
    """
    

class QueryAnsweringTask(Base):
    """
    An Answering task that is the reaction to some Query answering task.
    
    In a lot of cases, such a task is also an Assertion task, e.g., in the following discourse:
    
    "How will the weather be tomorrow?"
    "It is going to rain in the morning."
    
    However, sometimes this might be not the case, e.g., with counterquestions.
    """
    

class QueryEngine(Base):
    """
    A Query Engine is a Software that can answer some queries.
    """
    

class RaspberryJamJar(Base):
    """
    A jar which holds, or at least is intended to usually hold, raspberry jam.
    """
    

class Reaching(Base):
    """
    A task in which an Agent moves one or more of its arms towards a location or object.
    """
    

class Retracting(Base):
    """
    A task in which an Agent moves its arms away from a location.
    """
    

class Reasoner(Base):
    """
    A Reasoner is some Software that can infer new, implicit knowlegde from explicitly stated knowledge.
    
    This definition is broad and we consider any System fitting the above description as reasoners. For example, the following can be seen as Reasoners:
    * A simulation, where the explicit knowledge corresponds to the initial situation, and the implicit knowlegde corresponds to the situation that is derived from that by simulating some unfolding processes.
    * A machine learning algorithm, e.g., an image classifier: The explicit knowledge is the visual content of a picture (even down to the pixel), the implicit knowledge is the derived classification.
    * A logic based rule engine, where initial facts are the explicit knowledge, and derived facts are the implicit knowledge.
    """
    

class RecipientRole(Base):
    """
    A role which classifies an agent who receives an object modified or created by an action.
    """
    

class RecordedEpisode(Base):
    """
    An episode which has been recorded.
    """
    

class RedColor(Base):
    """
    A color region with dominant red color.
    """
    

class Reification(Base):
    """
    A description that *describes* a formal entity.
    """
    

class RelationalDatabase(Base):
    """
    A Relational Database is a Database based on the relational model of data, which organizes data into one or more tables (or "relations") of columns and rows, with a unique key identifying each row.
    
    Source: https://en.wikipedia.org/wiki/Relational_database
    """
    

class RelationalQueryLanguage(Base):
    """
    A Query Language that is designed for communication with some Relational Database.
    """
    

class RelevantPart(Base):
    """
    Features that are relevant parts of their host, like a bump or an edge.
    """
    

class Remembering(Base):
    """
    A Mental task in which the Agent recalls a record of a previous state of affairs in the world.
    
    The Agent must have witnessed and memorized this state of affairs in order to record it. Remembering commits itself to accuracy: the Agent attempts to reconstruct as accurate a record as it can. Note, this does not mean the Agent will communicate the recollection accurately.
    
    The difference to Memory retrieval is that for this Task, we are concerned with knowledge about a previous world state. Memory Retrieval is more general in the sense that it also includes the retrieval of learned facts and rules.
    """
    

class Retrospecting(Base):
    """
    A Mental task in which an Agent endeavors to construct a representation of a past state of affairs of the world it is embodied in.
    Done by analogy with Prospecting. Currently mono-subcategory, but perhaps we might find more.
    
    As an example, a kind of Abductive reasoning would fit here: reconstruction, in which the agent attempts to create a representation of a past state of affairs which the agent has not actually observed, based on traces and clues surviving to the present.
    """
    

class RemovedObject(Base):
    """
    An object that is removed from another.
    """
    

class Replanning(Base):
    """
    A mental task, in which an agent reconfigures some plan that has been put together before.
    """
    

class RevoluteJoint(Base):
    """
    A hinge joint that rotates along an axis and has a limited range specified by the upper and lower limits.
    """
    

class PhysicalPlace(Base):
    """
    A physical object that is inherently located; for example, a water area.
    """
    

class Surface(Base):
    """
    The outside part or uppermost layer of something.
    """
    

class Rubbing(Base):
    """
    The motion of an object sliding along the surface of another, for example, to clean the surface.
    """
    

class Scene(Base):
    """
    Scenes are Situations which interpret a State in terms of its conformance to some qualitative, image schematic description. I.e., the scene is described in terms of qualitative functional and spatial relations existing between participating objects.
    """
    

class Theory(Base):
    """
    A Theory is a Description that represents a set of assumptions for describing something, usually general. Scientific, philosophical, and commonsense theories can be included here.
    This class can also be used to act as 'naturalized reifications' of logical theories (of course, they will be necessarily incomplete in this case, because second-order entities are represented as first-order ones).
    """
    

class SelectedObject(Base):
    """
    An object chosen as the result of some selection task.
    """
    

class Selecting(Base):
    """
    A Task where an Agent decides between two or more options.
    """
    

class SelectingItem(Base):
    """
    A task in which an Agent selects some object to use for a subsequent task.
    """
    

class SelfReflection(Base):
    ...
    

class Serving(Base):
    """
    The task in which the agent delivers an object to a physical agent
    """
    

class SettingGripper(Base):
    """
    A task by which an Agent arranges one/some/all of its grippers in some configuration.
    """
    

class SixDPose(Base):
    """
    A point in three dimensional space, given as translation in a reference coordinate system, and an orientation of a coordinate system centered at that point relative to the reference coordinate system.
    """
    

class Sharpness(Base):
    """
    The quality of having a thin edge or point that can cut something or make a hole into something. It is worth to note here that the social aspect of sharpness such as the quality of being clear, intelligent etc is not considered as sharpness according to this definition.
    """
    

class Simulating(Base):
    """
    A Mental task in which the Agent endeavours to create representations of a sequence of states of affairs in the world. Simulation commits itself to some degree of transition accuracy: supposing the actual state of the world was the initial state of the simulation, the world state and simulation state should evolve to some degree similarly.
    
    Simulation does not commit itself to state accuracy: the initial state of the simulation is not constrained to be faithful to the actual state of the world in which the Agent is embodied. Counterfactual simulation ("what would happen if--?") is possible.
    """
    

class SimulationReasoner(Base):
    """
    A Simulation-based Reasoner is a simulation that is used as a reasoner, where the explicit knowledge corresponds to the initial situation, and the implicit knowlegde corresponds to the situation that is derived from that by simulating some unfolding processes.
    """
    

class Set(Base):
    ...
    

class Size(Base):
    """
    The magnitude or dimension of a thing which can be measured as length, width, height, diameter, perimeter, area, volume.
    """
    

class Slicing(Base):
    """
    A particular kind of cutting where the goal is to produce slices from some solid object.
    """
    

class Sluggishness(Base):
    """
    A description of sluggish behavior.
    """
    

class SocialState(Base):
    """
    A State which describes how Agents relate to each other.
    
    One can argue that any Social state is a Physical state, since anything the Agents may use to construct a social relationship is made of physical things. The difference is that the physical support of the social relationships is not important here, what matters instead is the nature and content of the social relations, regardless of how they are physically realized.
    Note here a distinction: the Agents involved must be able to communicate. This is because while it is often assumed that an Agent has or aspires to have similar cognitive capacities to a human, This need not be the case; in particular, not all Agents need to maintain theories of mind about each other and therefore not all Agents need to communicate with each other. It is hard to see what sort of meaning Social concepts might have to such Agents, since Social concepts are all about shared constructions.
    
    Note also that the DUL SocialAgent class is not an appropriate restriction however. SocialAgent is one that exists by agreement of PhysicalAgents. For example, a corporation or a nation are SocialAgents. An Agent with the capability to engage socially is not necessarily a DUL SocialAgent.
    """
    

class SoftwareConfiguration(Base):
    ...
    

class SocialAgent(Base):
    """
    Any individual whose existence is granted simply by its social communicability and capability of action (through some PhysicalAgent).
    """
    

class SoftwareLibrary(Base):
    ...
    

class SoupPot(Base):
    """
    A pot which holds, or at least is intended to usually hold, soup.
    """
    

class SourceMaterialRole(Base):
    """
    A role classifying a substance or object that enters a process of transformation intended to create some other object.
    """
    

class SphereShape(Base):
    """
    A round solid figure with every point on its surface equidistant from its centre.
    """
    

class Standing(Base):
    """
    A motion by which an Agent arranges its body in an upright configuration. Typically, it makes sense to speak of standing for bodies where some limbs are dedicated to moving the whole body while some limbs are used for manipulation of other objects.
    """
    

class StaticFrictionAttribute(Base):
    """
    Friction between two touching objects that do not move relative to each other.
    """
    

class Status(Base):
    """
    A role that can be played by some parameter which indicates the state of affairs of some entity, e.g. a flag describing the outcome of an action in terms of success or failure, or an indicator of whether a device is turned on or off.
    """
    

class StatusFailure(Base):
    """
    A status indicating the failure during some workflow execution.
    """
    

class StimulusRole(Base):
    """
    A role classifying an object that is perceived by some agent and thus triggers some reaction (e.g., a perception event).
    """
    

class Stirring(Base):
    """
    A task in which an agent dissolves small particles like sugar or salt in fluid
    """
    

class Storage(Base):
    """
    The disposition of an object (the container) to store other objects. Storage of an object would facilitate several objectives; such as to store objects in a safe or usual place, to prevent the substances e.g. prevention of milk going bad by storing them in a refrigrator.
    """
    

class Stove(Base):
    """
    An appliance for cooking or heating.
    """
    

class StructuralDesign(Base):
    """
    A design of an object which describes its stability, strength and rigidity, and considers the way in which parts of an object are arranged. 
            
    """
    

class TaskInvocation(Base):
    """
    An elementary workflow consisting in the invocation of one single task. It is used as a descriptive context inside of which factual bindings are valid between the task's arguments and other entities (such as the "local variables" of a larger workflow).
    """
    

class SuccessDiagnosis(Base):
    """
    A diagnosis of the fullfilment of a goal that motivates the behavior of a system.
    """
    

class Successfulness(Base):
    """
    A description of a situation with a goal that was achieved by some system.
    """
    

class SugarDispenser(Base):
    """
    A dispenser used to hold, or at least is intended to usually hold and serve, sugar.
    """
    

class SupportState(Base):
    """
    Classifies States in which an object is not able to move under gravity because of its placement relative to some other object.
    """
    

class Supporter(Base):
    """
    A role classifying an object used to support others.
    """
    

class SupportTheory(Base):
    """
    An image schematic theory that describes the reified functional relation holding between two spatial objects x and y, such that x physically supports y in the presence of gravity; x and y need not be in contact. An example of such an expression is "The bowl is on the table".
    
    This is also known as FunctionalControlExternal in GUM (Bateman et al. 2010).
    
    Let xL, xR be objects filling the locatum, relatum roles of this schema. Then one can infer that xL isSupportedBy xR.
    """
    

class SupportedObject(Base):
    """
    An object that is supported by some supporter.
    """
    

class SymbolicReasoner(Base):
    """
    A Symbolic Reasoner, is a piece of software able to infer logical consequences from a set of asserted facts or axioms.
    
    Source: https://en.wikipedia.org/wiki/Semantic_reasoner
    """
    

class TableFork(Base):
    """
    A regular size fork used for eating meals, e.g. for Spaghetti.
    """
    

class TableKnife(Base):
    """
    A knife used for eating or speading butter on bread.
    """
    

class TableSpoon(Base):
    """
    A spoon usually refered as larger spoon used for serving. However in some regions it is the largest spoon used for eating.
    """
    

class TeaSpoon(Base):
    """
    A spoon relatively smaller in size used for string a cup of tea or measuring a volume. Specifically for cooking or medicine purposes, the volume is defined as 5ml.
    """
    

class Tap(Base):
    """
    A component designed to control the flow of water from a pipe.
    """
    

class Tapping(Base):
    """
    A motion, usually repeated several times, for example, to probe the surface of an object.
    """
    

class Taxis(Base):
    """
    An innate behavioural response such as the knee-jerk reflex or the sucking reflex of human infants.
    """
    

class TechnicalDiagnosis(Base):
    """
    A functional diagnosis of a technical system.
    """
    

class Temperature(Base):
    """
    The heat present in an object.
    """
    

class TemperatureRegion(Base):
    """
    Encodes the temperature of an object.
    """
    

class Tempering(Base):
    """
    The disposition of an object (the tool) to change the temperature of others.
    """
    

class TemperingByCooling(Base):
    """
    A disposition of an object to change others by removing heat energy from them.
    """
    

class ThinkAloud(Base):
    """
    A task in which an Agent, while in the course of performing some other task(s), reports on their own decision processes that guide this other task(s) for the benefit of an outside observer.
    """
    

class ThinkAloudActionTopic(Base):
    """
    A topic used when an Agent states what they are doing.
    """
    

class ThinkAloudGeneralKnowledgeTopic(Base):
    """
    A topic used when an Agent states general knowledge they have.
    """
    

class ThinkAloudKnowledgeTopic(Base):
    """
    A topic used when an Agent states some item of knowledge. This knowledge can be general, or specific to the environment and task at hand.
    """
    

class ThinkAloudObstructionTopic(Base):
    """
    A topic used when an Agent describes some state of affairs that prevents them from performing an action.
    """
    

class ThinkAloudOpinionTopic(Base):
    """
    A topic used when an Agent expresses an opinion about the action they perform or the environment they are in.
    """
    

class ThinkAloudPerceptionTopic(Base):
    """
    A topic used when an Agent describes what they currently perceive.
    """
    

class ThinkAloudPlanTopic(Base):
    """
    A topic used when an Agent describes what they intend to do. Note, this is not about describing the process through which this plan was constructed; that is covered by the MetaCognitionPlanningTopic.
    """
    

class ThinkAloudSceneKnowledgeTopic(Base):
    """
    A topic used when an Agent describes what they know about their environment, including knowledge of world states that they do not currently perceive.
    """
    

class Threshold(Base):
    """
    A role played by a parameter which indicates some value that, when crossed, triggers some condition.
    """
    

class Throwing(Base):
    """
    A task in which an Agent imparts momentum to an object before releasing it so that it flies for some distance unsupported.
    """
    

class TimeRole(Base):
    """
    A role filled by a description of the location in time and/or duration of an event or object.
    """
    

class Transporting(Base):
    """
    A task by which an Agent carries an item from a source to a destination location.
    """
    

class Triplestore(Base):
    """
    A Triplestore or RDF store is a purpose-built database for the storage and retrieval of triples through semantic queries.
    """
    

class Turning(Base):
    """
    A motion by which an agent changes which way their body faces.
    """
    

class UnavailableSoftware(Base):
    """
    A description of a situation where some software dependency is not available.
    """
    

class Unsuccessfulness(Base):
    """
    A description of a situation with a goal that was not or not fully achieved by some system.
    """
    

class VideoData(Base):
    """
    An information object containing data for audio-visual modalities.
    """
    

class Wardrobe(Base):
    """
    A cupboard which is typically used for hanging clothes.
    """
    

class WaterBottle(Base):
    """
    A bottle which holds, or at least is intended to usually hold, water.
    """
    

class ThreeDPosition(Base):
    """
    A point in three dimensional space, given as translation.
    """
    

class NonmanifestedSituation(Base):
    ...
    

class Abstract(Base):
    """
    Any Entity that cannot be located in space-time. E.g. mathematical entities: formal semantics elements, regions within dimensional spaces, etc.
    """
    

class Amount(Base):
    """
    A quantity, independently from how it is measured, computed, etc.
    """
    

class ChemicalObject(Base):
    ...
    

class Classification(Base):
    """
    A special kind of Situation that allows to include time indexing for the classifies relation in situations. For example, if a Situation s 'my old cradle is used in these days as a flower pot' isSettingFor the entity 'my old cradle' and the TimeIntervals '8June2007' and '10June2007', and we know that s satisfies a functional Description for aesthetic objects, which defines the Concepts 'flower pot' and 'flower', then we also need to know what concept classifies 'my old cradle' at what time.
    In order to solve this issue, we need to create a sub-situation s' for the classification time: 'my old cradle is a flower pot in 8June2007'. Such sub-situation s' isPartOf s.
    """
    

class Collective(Base):
    """
    A Collection whose members are agents, e.g. "the nurses", "the Italian rockabilly fans".
    Collectives, facon de parler, can act as agents, although they are not assumed here to be agents (they are even disjoint from the class SocialAgent). This is represented by admitting collectives in the range of the relations having Agent in their domain or range.
    """
    

class CollectiveAgent(Base):
    """
    A SocialAgent that is actedBy agents that are (and act as) members of a Collective. A collective agent can have roles that are also roles of those agents.
    For example, in sociology, a 'group action' is the situation in which a number of people (that result to be members of a collective) in a given area behave in a coordinated way in order to achieve a (often common) goal. The Agent in such a Situation is not single, but a CollectiveAgent (a Group). This can be generalized to the notion of social movement, which assumes a large Community or even the entire Society as agents.
    The difference between a CollectiveAgent and an Organization is that a Description that introduces a CollectiveAgent is also one that unifies the corresponding Collective. In practice, this difference makes collective agents 'less stable' than organizations, because they have a dedicated, publicly recognizable Description that is conceived to introduce them.
    """
    

class Community(Base):
    ...
    

class Contract(Base):
    """
    (The content of) an agreement between at least two agents that play a Party Role, about some contract object (a Task to be executed).
    """
    

class Diagnosis(Base):
    """
    A Description of the Situation of a system, usually applied in order to control a normal behaviour, or to explain a notable behavior (e.g. a functional breakdown).
    """
    

class FunctionalSubstance(Base):
    ...
    

class Goal(Base):
    """
    The Description of a Situation that is desired by an Agent, and usually associated to a Plan that describes how to actually achieve it
    """
    

class Group(Base):
    """
    A CollectiveAgent whose acting agents conceptualize a same SocialRelation .
    """
    

class InformationEntity(Base):
    """
    A piece of information, be it concretely realized or not. It is a catchall class, intended to bypass the ambiguities of many data or text that could denote either a an expression or a concrete realization of that expression.
    In a semiotic model, there is no special reason to distinguish between them, however we may want to distinguish between a pure information content (e.g. the 3rd Gymnopedie by Satie), and its possible concrete realizations as a music sheet, a piano execution, the reproduction of the execution, its publishing as a record, etc.).
    """
    

class LocalConcept(Base):
    """
    A Concept that isDefinedIn exactly 1 Description. For example, the Concept 'coffee' in a 'preparesCoffee' relation can be defined in that relation, and for all other Description(s) that use it, the isConceptUsedIn property should be applied. Notice therefore that not necessarily all Concept(s) isDefinedIn exactly 1 Description.
    """
    

class Method(Base):
    """
    A method is a Description that defines or uses concepts in order to guide carrying out actions aimed at a solution with respect to a problem. 
    It is different from a Plan, because plans could be carried out in order to follow a method, but a method can be followed by executing alternative plans.
    """
    

class Narrative(Base):
    ...
    

class Norm(Base):
    """
    A social norm.
    """
    

class ObjectAggregate(Base):
    """
    An aggregate of distributed objects, members of a same Collection, e.g. the stars in a constellation, the parts of a car, the employees of a company, the entries from an encyclopedia, the concepts expressed in a speech, etc.
    It cannot be defined by means of an equivalence axiom, because it'd require the same Collection for all members, an axiom that cannot be expressed in OWL.
    """
    

class Organism(Base):
    """
    A physical objects with biological characteristics, typically that organisms can self-reproduce.
    """
    

class Organization(Base):
    """
    An internally structured, conventionally created SocialAgent, needing a specific Role and Agent that plays it, in order to act.
    """
    

class Parthood(Base):
    """
    A special kind of Situation that allows to include time indexing for the hasPart relation in situations. 
    For example, if a Situation s 'finally, my bike has a luggage rack' isSettingFor the entity 'my bike' and the TimeIntervals 'now', or more specifically '29March2021', we need to have a time-index the part relation. With Parthood, we use includesWhole and includesPart properties.
    This can be done similarly for other arguments of parthood, e.g. location, configuration, topology, etc.
    Concerning the possible property characteristics reused from mereology (transitivity, asymmetry, reflexivity), they need to be implemented by means of rules (or, in a limited way, property chains using the binary hasPart or hasProperPart properties).
    A key is also added to ensure identification constraints of time-indexed parthood.
    """
    

class Pattern(Base):
    """
    Any invariance detected from a dataset, or from observation; also, any invariance proposed based on top-down considerations.
    E.g. patterns detected and abstracted by an organism, by pattern recognition algorithms, by machine learning techniques, etc.
    An occurrence of a pattern is an 'observable', or detected Situation
    """
    

class Person(Base):
    """
    Persons in commonsense intuition, which does not apparently distinguish between either natural or social persons.
    """
    

class Personification(Base):
    """
    A social entity with agentive features, but whose status is the result of a cultural transformation from e.g. a PhysicalObject, an Event, an Abstract, another SocialObject, etc. For example: the holy grail, deus ex machina, gods, magic wands, etc.
    """
    

class PhysicalArtifact(Base):
    """
    Any PhysicalObject that isDescribedBy a Plan .
    This axiomatization is weak, but allows to talk of artifacts in a very general sense, i.e. including recycled objects, objects with an intentional functional change, natural objects that are given a certain function, even though they are not modified or structurally designed, etc. PhysicalArtifact(s) are not considered disjoint from PhysicalBody(s), in order to allow a dual classification when needed. E.g.,
    FunctionalSubstance(s) are included here as well.
    Immaterial (non-physical) artifacts (e.g. texts, ideas, cultural movements, corporations, communities, etc. can be modelled as social objects (see SocialObject), which are all 'artifactual' in the weak sense assumed here.
    """
    

class Place(Base):
    """
    Socially or cognitively dependent locations: political geographic entities (Rome, Lesotho), and non-material locations determined by the presence of other entities ("the area close to Rome") or of pivot events or signs ("the area where the helicopter fell"), as well as identified as complements to other entities ("the area under the table"), etc. 
    In this generic sense, a Place is a 'dependent' location. For 'non-dependent' locations, cf. the PhysicalPlace class. For an abstract (dimensional) location, cf. the SpaceRegion class.
    """
    

class PlanExecution(Base):
    """
    Plan executions are situations that proactively satisfy a plan. Subplan executions are proper parts of the whole plan execution.
    """
    

class Project(Base):
    """
    A Plan that defines Role(s), Task(s), and a specific structure for tasks to be executed in relation to goals to be achieved, in order to achieve the main goal of the project. In other words, a project is a plan with a subgoal structure and multiple roles and tasks.
    """
    

class Right(Base):
    """
    A legal position by which an Agent is entitled to obtain something from another Agent , under specified circumstances, through an enforcement explicited either in a Law, Contract , etc.
    """
    

class SocialPerson(Base):
    """
    A SocialAgent that needs the existence of a specific NaturalPerson in order to act (but the lifetime of the NaturalPerson has only to overlap that of the SocialPerson).
    """
    

class SpatioTemporalRegion(Base):
    ...
    

class TimeIndexedRelation(Base):
    """
    A Situation that includes a time indexing in its setting, so allowing to order any binary relation (property) with time.
    """
    

class TimeInterval(Base):
    """
    Any Region in a dimensional space that aims at representing time.
    """
    

class TypeCollection(Base):
    """
    A Collection whose members are the maximal set of individuals that share the same (named) type, e.g. "the gem stones", "the Italians".
    This class is very useful to apply a variety of the so-called "ClassesAsValues" design pattern, when it is used to talk about the extensional aspect of a class. An alternative variety of the pattern applies to the intensional aspect of a class, and the class Concept should be used instead.
    """
    

class UnitOfMeasure(Base):
    """
    Units of measure are conceptualized here as parameters on regions, which can be valued as datatype values.
    """
    

class WorkflowExecution(Base):
    ...
    

