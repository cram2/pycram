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
    
class associated_with(BaseProperty):
    """
    A catch-all object property, useful for alignment and querying purposes.
    It is declared as both transitive and symmetric, in order to reason an a maximal closure of associations between individuals.
    """
    
class has_location(BaseProperty):
    """
    A generic, relative spatial location, holding between any entities. E.g. 'the cat is on the mat', 'Omar is in Samarcanda', 'the wound is close to the femural artery'.
    For 'absolute' locations, see SpaceRegion
    """
    
class affects(BaseProperty):
    """
    Simple relationship between two actions to express that a variation in the course or outcome of the subject (the affector) would have resulted in a variation in the object (the affectee), e.g., a planning task that sets parameters such as goal position affects the subsequently executed pick-and-place task that uses that parameter.
    Simple relationship between two actions to express that a variation in the course or outcome of the subject (the affector) would have resulted in a variation in the object (the affectee), e.g., a planning task that sets parameters such as goal position affects the subsequently executed pick-and-place task that uses that parameter.
    """
    
class precedes(BaseProperty):
    """
    A relation between entities, expressing a 'sequence' schema. 
    E.g. 'year 1999 precedes 2000', 'deciding what coffee to use' precedes 'preparing coffee', 'World War II follows World War I', 'in the Milan to Rome autoroute, Bologna precedes Florence', etc.
    It can then be used between tasks, processes, time intervals, spatially locate objects, situations, etc. 
    Subproperties can be defined in order to distinguish the different uses.
    """
    
class is_affected_by(BaseProperty):
    """
    Simple relationship between two actions to express that a variation in the course or outcome of the object (the affector) would have resulted in a variation in the subject (the affectee), e.g., a planning task that sets parameters such as goal position affects the subsequently executed pick-and-place task that uses that parameter.
    Simple relationship between two actions to express that a variation in the course or outcome of the object (the affector) would have resulted in a variation in the subject (the affectee), e.g., a planning task that sets parameters such as goal position affects the subsequently executed pick-and-place task that uses that parameter.
    """
    
class affordance_defines(BaseProperty):
    """
    A relation between an Affordance and a Concept (often an EventType).
    A relation between an Affordance and a Concept (often an EventType).
    """
    
class defines(BaseProperty):
    """
    A relation between a Description and a Concept, e.g. a Workflow for a governmental Organization defines the Role 'officer', or 'the Italian Traffic Law defines the role Vehicle'.
    """
    
class is_defined_in_affordance(BaseProperty):
    """
    A relation between a Concept and an Affordance.
    A relation between a Concept and an Affordance.
    """
    
class affordance_defines_task(BaseProperty):
    """
    A relation between an Affordance and a Task
    A relation between an Affordance and a Task
    """
    
class defines_task(BaseProperty):
    """
    A relation between a description and a task, e.g. the recipe for a cake defines the task 'boil'.
    """
    
class is_task_defined_in_affordance(BaseProperty):
    """
    A relation between a Task and an Affordance, such that the task is defined in terms of using the affordance.
    A relation between a Task and an Affordance, such that the task is defined in terms of using the affordance.
    """
    
class affords_bearer(BaseProperty):
    """
    Relates a disposition to the bearer role defined by the affordance describing the disposition.
    Relates a disposition to the bearer role defined by the affordance describing the disposition.
    """
    
class affords_concept(BaseProperty):
    """
    A relation between a disposition and a concept defined in the affordance that describes the disposition.
    A relation between a disposition and a concept defined in the affordance that describes the disposition.
    """
    
class is_bearer_afforded_by(BaseProperty):
    """
    Relates a disposition to the bearer role defined by the affordance describing the disposition.
    Relates a disposition to the bearer role defined by the affordance describing the disposition.
    """
    
class is_described_by(BaseProperty):
    """
    The relation between an Entity and a Description: a Description gives a unity to a Collection of parts (the components), or constituents, by assigning a Role to each of them in the context of a whole Object (the system).
    A same Entity can be given different descriptions, for example, an old cradle can be given a unifying Description based on the original aesthetic design, the functionality it was built for, or a new aesthetic functionality in which it can be used as a flower pot.
    """
    
class defines_bearer(BaseProperty):
    """
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the bearer when the affordance is manifested.
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the bearer when the affordance is manifested.
    """
    
class is_concept_afforded_by(BaseProperty):
    """
    A relation between a disposition and a concept defined in the affordance that describes the disposition.
    A relation between a disposition and a concept defined in the affordance that describes the disposition.
    """
    
class affords_performer(BaseProperty):
    """
    Relates a disposition to the performer role defined by the affordance describing the disposition.
    Relates a disposition to the performer role defined by the affordance describing the disposition.
    """
    
class is_performer_afforded_by(BaseProperty):
    ...
    
class defines_performer(BaseProperty):
    """
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the performer when the affordance is manifested.
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the performer when the affordance is manifested.
    """
    
class affords_setpoint(BaseProperty):
    """
    Relates a disposition to the setpoint parameter defined by the affordance describing the disposition.
    Relates a disposition to the setpoint parameter defined by the affordance describing the disposition.
    """
    
class is_setpoint_afforded_by(BaseProperty):
    """
    Relates a disposition to the setpoint parameter defined by the affordance describing the disposition.
    Relates a disposition to the setpoint parameter defined by the affordance describing the disposition.
    """
    
class defines_setpoint(BaseProperty):
    """
    Defines the dedicated goal region of a description.
    Defines the dedicated goal region of a description.
    """
    
class affords_task(BaseProperty):
    """
    Relates a disposition to the task defined by the affordance describing the disposition.
    Relates a disposition to the task defined by the affordance describing the disposition.
    """
    
class is_task_afforded_by(BaseProperty):
    """
    Relates a disposition to the task defined by the affordance describing the disposition.
    Relates a disposition to the task defined by the affordance describing the disposition.
    """
    
class affords_trigger(BaseProperty):
    """
    Relates a disposition to the trigger role defined by the affordance describing the disposition.
    Relates a disposition to the trigger role defined by the affordance describing the disposition.
    """
    
class is_trigger_afforded_by(BaseProperty):
    """
    Relates a disposition to the trigger role defined by the affordance describing the disposition.
    Relates a disposition to the trigger role defined by the affordance describing the disposition.
    """
    
class defines_trigger(BaseProperty):
    """
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the trigger when the affordance is manifested.
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the trigger when the affordance is manifested.
    """
    
class after(BaseProperty):
    """
    A relation between entities, expressing a 'sequence' schema where one of the entities strictly ends before the other one.
    """
    
class answers(BaseProperty):
    ...
    
class relates_to_another_role(BaseProperty):
    """
    Simple top-level property for relations between two roles.
    Simple top-level property for relations between two roles.
    """
    
class has_answer(BaseProperty):
    """
    The relation between a message and its answer.
    The relation between a message and its answer.
    """
    
class before(BaseProperty):
    """
    A relation between entities, expressing a 'sequence' schema where one of the entities strictly ends before the other one.
    """
    
class causes(BaseProperty):
    ...
    
class is_reaction_to(BaseProperty):
    """
    Simple relationship between two actions to express that the subject (the reaction) would not have occured if it were not for the object (the cause), e.g., a Communication Action classified as an Answering Task is a reaction to another Communication Task classified as a Query Task and would not have occured without the other. An example without Agents involved would be some domino stone would not have toppled without the first one toppling.
    
    When Agents are involved, the relation might be seen as an abstraction of the execution of some plan that arises from changing the agents goal that is due to perceiving the cause. However, currently it is unclear how to model such a pattern and therefore not included in SOMA.
    
    This relation is seen as transitive.
    Simple relationship between two actions to express that the subject (the reaction) would not have occured if it were not for the object (the cause), e.g., a Communication Action classified as an Answering Task is a reaction to another Communication Task classified as a Query Task and would not have occured without the other. An example without Agents involved would be some domino stone would not have toppled without the first one toppling.
    
    When Agents are involved, the relation might be seen as an abstraction of the execution of some plan that arises from changing the agents goal that is due to perceiving the cause. However, currently it is unclear how to model such a pattern and therefore not included in SOMA.
    
    This relation is seen as transitive.
    """
    
class causes_transition(BaseProperty):
    """
    A Transition between two Situations is always the result of some Event, and the causesTransition relation should be used to record the causal relation from the Event to the Transition.
    """
    
class co_occurs(BaseProperty):
    """
    A schematic relation between any events that also implies that one event is temporally contained in the other.
    
    Sub-properties are used to distinct between different cases of event endpoint relations that hold for different types of co-occurance.
    """
    
class contains(BaseProperty):
    """
    A schematic relation asserting containment, understood in a very broad sense, by one Entity of another. The relation is defined with domain and range of maximum generality, as it is possible to construe containment to apply between Events, between Objects, between Qualities and so on. Care should be taken when using it that the construal of containment makes sense and is useful. If a clearer relation expresses the connection between two Entities, use that relation instead. For example, rather than saying an Event contains an Object, it is probably better to say the Event has that Object as a participant. More specific versions of this relation exist, e.g. containsEvent, so it is likely that there will be few situations where it should be used itself. However, by acting as a superproperty to several relations, it captures a core similarity between these and enables taxonomy-based similarity metrics.
    """
    
class contains_event(BaseProperty):
    """
    `A contains event B` means that A strictly starts before, and ends after B, i.e. B is wholly contained in A.
    """
    
class contains_object(BaseProperty):
    """
    A spatial relation holding between a container, and objects it contains.
    A spatial relation holding between a container, and objects it contains.
    """
    
class is_location_of(BaseProperty):
    """
    A generic, relative localization, holding between any entities. E.g. 'Rome is the seat of the Pope', 'the liver is the location of the tumor'.
    For 'absolute' locations, see SpaceRegion
    """
    
class is_inside_of(BaseProperty):
    """
    A spatial relation holding between an object (the container), and objects it contains.
    A spatial relation holding between an object (the container), and objects it contains.
    """
    
class covers_object(BaseProperty):
    """
    A relationship from an object (the coverer) that blocks access to another or its interior (the coveree).
    A relationship from an object (the coverer) that blocks access to another or its interior (the coveree).
    """
    
class interacts_with(BaseProperty):
    """
    A relation between objects that interact with each other.
    A relation between objects that interact with each other.
    """
    
class is_covered_by_object(BaseProperty):
    """
    A relation from an object (the coveree) which is itself, or has its interior, prevented from being accessed from outside by a coverer.
    A relation from an object (the coveree) which is itself, or has its interior, prevented from being accessed from outside by a coverer.
    """
    
class defines_role(BaseProperty):
    """
    A relation between a description and a role, e.g. the recipe for a cake defines the role 'ingredient'.
    """
    
class is_bearer_defined_in(BaseProperty):
    """
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the bearer when the affordance is manifested.
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the bearer when the affordance is manifested.
    """
    
class defines_event_type(BaseProperty):
    ...
    
class is_event_type_defined_in(BaseProperty):
    """
    A relation between an event type and a description, e.g. an event that is described by the Affordance of an object to be cut with a knife.
    
    The distinction to 'is task defined in' is necessary to let Dispositions and Affordances not only describe which tasks might be afforded by objects, but also whihc processes (where there is no agent). For example, the fall of a knife from a shelf slicing a loaf of bread on impact is , in the absence of an executing agent, not a task but merely a process, the possibility of which is nevertheless described by the dispositions of the knife and the loaf.
    A relation between an event type and a description, e.g. an event that is described by the Affordance of an object to be cut with a knife.
    
    The distinction to 'is task defined in' is necessary to let Dispositions and Affordances not only describe which tasks might be afforded by objects, but also whihc processes (where there is no agent). For example, the fall of a knife from a shelf slicing a loaf of bread on impact is , in the absence of an executing agent, not a task but merely a process, the possibility of which is nevertheless described by the dispositions of the knife and the loaf.
    """
    
class defines_input(BaseProperty):
    """
    The defined participant is an "input": 
    
    - some object existing at the beginning of a Task's execution, and that will be acted on during the execution of the task;
    - some region/value which informs the way in which the Task will be executed.
    The defined participant is an "input": 
    
    - some object existing at the beginning of a Task's execution, and that will be acted on during the execution of the task;
    - some region/value which informs the way in which the Task will be executed.
    """
    
class defines_participant(BaseProperty):
    """
    A Description definesParticipant a Concept to classify participants in Events associated to that Description.
    
    The prototypical example is a Task, which is a concept to classify Actions (a form of Event). A Task may define several Roles, with which to classify participants in the event of that Task's execution.
    A Description definesParticipant a Concept to classify participants in Events associated to that Description.
    
    The prototypical example is a Task, which is a concept to classify Actions (a form of Event). A Task may define several Roles, with which to classify participants in the event of that Task's execution.
    """
    
class defines_output(BaseProperty):
    """
    Defines an "output" participant:
    
    - an Entity (Object or state of affairs) that exists as a result of the execution of a Task;
    - a Region/value that has been demarcated/computed as a result of the execution of a Task.
    Defines an "output" participant:
    
    - an Entity (Object or state of affairs) that exists as a result of the execution of a Task;
    - a Region/value that has been demarcated/computed as a result of the execution of a Task.
    """
    
class defines_parameter(BaseProperty):
    """
    A relation between a description and a parameter.
    A relation between a description and a parameter.
    """
    
class is_parameter_defined_in(BaseProperty):
    """
    A relation between a description and a parameter.
    A relation between a description and a parameter.
    """
    
class is_performer_defined_in(BaseProperty):
    """
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the performer when the affordance is manifested.
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the performer when the affordance is manifested.
    """
    
class defines_process(BaseProperty):
    """
    A relation between a description and a process type.
    A relation between a description and a process type.
    """
    
class is_process_defined_in(BaseProperty):
    """
    A relation between a process type and a description that defines it.
    A relation between a process type and a description that defines it.
    """
    
class is_setpoint_defined_in(BaseProperty):
    """
    Defines the dedicated goal region of a description.
    Defines the dedicated goal region of a description.
    """
    
class is_trigger_defined_in(BaseProperty):
    """
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the trigger when the affordance is manifested.
    Relates an affordance which is a relation between a bearer and a trigger, to the role of the trigger when the affordance is manifested.
    """
    
class derived_from(BaseProperty):
    """
    The (transitive) relation between an information object and another which it has been derived from.
    """
    
class describes_quality(BaseProperty):
    """
    Relates a description to a quality that it describes.
    Relates a description to a quality that it describes.
    """
    
class describes(BaseProperty):
    """
    The relation between a Description and an Entity : a Description gives a unity to a Collection of parts (the components), or constituents, by assigning a Role to each of them in the context of a whole Object (the system).
    A same Entity can be given different descriptions, for example, an old cradle can be given a unifying Description based on the original aesthetic design, the functionality it was built for, or a new aesthetic functionality in which it can be used as a flower pot.
    """
    
class is_quality_described_by(BaseProperty):
    """
    Relates a description to a quality it describes.
    Relates a description to a quality it describes.
    """
    
class directly_causes(BaseProperty):
    """
    Non-transitive version of "causes".
    Non-transitive version of "causes".
    """
    
class is_direct_reaction_to(BaseProperty):
    """
    Non-transitive version of "is reaction to".
    Non-transitive version of "is reaction to".
    """
    
class has_terminal_scene(BaseProperty):
    """
    A relation between StateTransitions and Scenes, which identifies the scene the transition is expected to end at.
    """
    
class includes_event(BaseProperty):
    """
    A relation between situations and events, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: the preparation of my coffee this morning included a burning of my fingers).
    """
    
class directly_derived_from(BaseProperty):
    """
    The relation between an information object and another which it has been derived from.
    """
    
class during(BaseProperty):
    """
    `A during B` means that B strictly starts before, and ends after A, i.e. A is wholly contained in B.
    """
    
class encapsulates(BaseProperty):
    """
    The relation between an Ordered element' and an 'Entity' it contains.
    The relation between an Ordered element' and an 'Entity' it contains.
    """
    
class encodes(BaseProperty):
    """
    The relation between two Information Objects that have the same meaning, but are formatted differently. E.g., a text written in UTF-8 encodes a text in a natural writing system (letters) and vice versa.
    The relation between two Information Objects that have the same meaning, but are formatted differently. E.g., a text written in UTF-8 encodes a text in a natural writing system (letters) and vice versa.
    """
    
class executes_motion(BaseProperty):
    """
    A relation between an motion process and a motion event type, e.g. 'lifting an object' executes the motion process 'lifting'.
    A relation between an motion process and a motion event type, e.g. 'lifting an object' executes the motion process 'lifting'.
    """
    
class is_occurrence_of(BaseProperty):
    """
    A relation between an event and an event type, e.g. 'taking the cup from the table' is an occurence of the motion 'approaching'.
    """
    
class is_executed_motion_in(BaseProperty):
    """
    A relation between an motion process and a motion event type, e.g. 'lifting an object' executes the motion process 'lifting'.
    A relation between an motion process and a motion event type, e.g. 'lifting an object' executes the motion process 'lifting'.
    """
    
class finished_by(BaseProperty):
    """
    `A finishes B` means that A ends exactly where B ends, and that B strictly starts before A.  As in "I finish my day by taking a shower".
    """
    
class finishes(BaseProperty):
    """
    `A finishes B` means that A ends exactly where B ends, and that B strictly starts before A.  As in "I finish my day by taking a shower".
    """
    
class first_member(BaseProperty):
    """
    A relation between a collection and a member of it that is least according to some ordering. This ordering can be arbitrary, i.e. the order in which Entities are recorded in the Collection.
    """
    
class gives_meaning_to(BaseProperty):
    """
    The relation between a System and Information Object that is given meaning to by said system, e.g., a Language might give meaning to some word, sentence, text, etc., but without the knowledge of said System (Language), the text will not make sense to a reader.
    The relation between a System and Information Object that is given meaning to by said system, e.g., a Language might give meaning to some word, sentence, text, etc., but without the knowledge of said System (Language), the text will not make sense to a reader.
    """
    
class is_given_meaning_by(BaseProperty):
    """
    The relation between an Information Object and a System that gives meaning to said object, e.g., a word, sentence, text, etc. might be given meaning by a Language and without the knowledge of said System (Language), the text will not make sense to a reader.
    The relation between an Information Object and a System that gives meaning to said object, e.g., a word, sentence, text, etc. might be given meaning by a Language and without the knowledge of said System (Language), the text will not make sense to a reader.
    """
    
class has_action(BaseProperty):
    """
    A relation from an Action to a component Action.
    A relation from an Action to a component Action.
    """
    
class has_constituent(BaseProperty):
    """
    'Constituency' depends on some layering of  the world described by the ontology. For example, scientific granularities (e.g. body-organ-tissue-cell) or ontological 'strata' (e.g. social-mental-biological-physical) are  typical layerings. 
    Intuitively, a constituent is a part belonging to a lower layer. Since layering is actually a partition of the world described by the ontology, constituents are not properly classified as parts, although this kinship can be intuitive for common sense.
    A desirable advantage of this distinction is that we are able to talk e.g. of physical constituents of non-physical objects (e.g. systems), while this is not possible in terms of parts.
    Example of are the persons constituting a social system, the molecules constituting a person, the atoms constituting a river, etc. 
    In all these examples, we notice a typical discontinuity between the constituted and the constituent object: e.g. a social system is conceptualized at a different layer from the persons that constitute it, a person is conceptualized at a different layer from the molecules that constitute them, and a river is conceptualized at a different layer from the atoms that constitute it.
    """
    
class has_alteration_result(BaseProperty):
    """
    Relates an action that alters an object to the region that the alteration reached during the action.
    Relates an action that alters an object to the region that the alteration reached during the action.
    """
    
class has_region(BaseProperty):
    """
    A relation between entities and regions, e.g. 'the number of wheels of that truck is 12', 'the time of the experiment is August 9th, 2004', 'the whale has been localized at 34 degrees E, 20 degrees S'.
    """
    
class is_alteration_result_of(BaseProperty):
    """
    Relates an action that alters an object to the region that the alteration reached during the action.
    Relates an action that alters an object to the region that the alteration reached during the action.
    """
    
class has_binding(BaseProperty):
    """
    Asserts that in a context described by Description, a Binding relation holds.
    Asserts that in a context described by Description, a Binding relation holds.
    """
    
class has_part(BaseProperty):
    """
    A schematic relation between any entities, e.g. 'the human body has a brain as part', '20th century contains year 1923', 'World War II includes the Pearl Harbour event'.
    
    Parthood should assume the basic properties of mereology: transitivity, antisymmetry, and reflexivity (propert Parthood of course misses reflexivity). 
    However, antisymmetry is not supported in OWL2 explicitly, therefore DUL has to adopt one of two patterns:
    1) dropping asymmetry axioms, while granting reflexivity: this means that symmetry is not enforced, but permitted for the case of reflexivity. Of course, in this way we cannot prevent symmetric usages of hasPart;
    2) dropping the reflexivity axiom, and enforce asymmetry: in this case, we would prevent all symmetric usages, but we loose the possibility of enforcing reflexivity, which is commonsensical in parthood.
    In DUL, we adopt pattern #1 for partOf, and pattern #2 for properPartOf, which seems a good approximation: due to the lack of inheritance of property characteristics, each asymmetric hasPropertPart assertion would also be a reflexive hasPart assertion (reflexive reduction design pattern).
    
    Subproperties and restrictions can be used to specialize hasPart for objects, events, etc.
    """
    
class has_binding_filler(BaseProperty):
    """
    Indicates that an Entity is described by a Binding, in that it is associated with the Role/Parameter that the Binding binds it to. The Binding is only valid in some descriptive context such as a Workflow or Narrative.
    
    Note, the filler can be a Role/Parameter as well, and there are two distinct interpretations, depending on whether the Binding is a RoleRoleBinding or a RoleFillerBinding. See the comments on Binding for more details.
    
    Only RoleFillerBindings can have general Entities as fillers. RoleRoleBindings can only connect to Roles or Parameters via this property.
    Indicates that an Entity is described by a Binding, in that it is associated with the Role/Parameter that the Binding binds it to. The Binding is only valid in some descriptive context such as a Workflow or Narrative.
    
    Note, the filler can be a Role/Parameter as well, and there are two distinct interpretations, depending on whether the Binding is a RoleRoleBinding or a RoleFillerBinding. See the comments on Binding for more details.
    
    Only RoleFillerBindings can have general Entities as fillers. RoleRoleBindings can only connect to Roles or Parameters via this property.
    """
    
class has_binding_role(BaseProperty):
    """
    Indicates that a Role/Parameter is going to be associated to some filler, or other Role/Parameter, by a Binding. The Binding is only valid in some descriptive context such as a Narrative or Workflow.
    Indicates that a Role/Parameter is going to be associated to some filler, or other Role/Parameter, by a Binding. The Binding is only valid in some descriptive context such as a Narrative or Workflow.
    """
    
class has_child_link(BaseProperty):
    """
    Relates a joint to the link it connects which is closer to the end of the kinematic chain.
    Relates a joint to the link it connects which is closer to the end of the kinematic chain.
    """
    
class is_child_link_of(BaseProperty):
    """
    Relates a joint to the link it connects which is closer to the end of the kinematic chain.
    Relates a joint to the link it connects which is closer to the end of the kinematic chain.
    """
    
class has_color(BaseProperty):
    """
    Relates an object to its color quality.
    Relates an object to its color quality.
    """
    
class has_quality(BaseProperty):
    """
    A relation between entities and qualities, e.g. 'Dmitri's skin is yellowish'.
    """
    
class is_color_of(BaseProperty):
    """
    Relates a color quality to the object the color belongs to.
    Relates a color quality to the object the color belongs to.
    """
    
class is_disposition_of(BaseProperty):
    """
    Associates a disposition quality to the object holding it.
    Associates a disposition quality to the object holding it.
    """
    
class has_end_link(BaseProperty):
    """
    Relates an object to kinematic components at the end of the kinematic chain.
    Relates an object to kinematic components at the end of the kinematic chain.
    """
    
class has_link(BaseProperty):
    """
    Relates an object to its kinematic components.
    Relates an object to its kinematic components.
    """
    
class is_end_link_of(BaseProperty):
    """
    Relates an object to kinematic components at the end of the kinematic chain.
    Relates an object to kinematic components at the end of the kinematic chain.
    """
    
class has_execution_state(BaseProperty):
    """
    A relation from an Action to its execution state.
    A relation from an Action to its execution state.
    """
    
class has_feature(BaseProperty):
    """
    Associates a physical object to one of its features.
    Associates a physical object to one of its features.
    """
    
class is_feature_of(BaseProperty):
    """
    Associates a feature to the physical object it belongs to.
    Associates a feature to the physical object it belongs to.
    """
    
class has_first_step(BaseProperty):
    """
    A relation from a Workflow to its first Task.
    A relation from a Workflow to its first Task.
    """
    
class has_step(BaseProperty):
    """
    A relation between a Workflow and a Task it contains.
    A relation between a Workflow and a Task it contains.
    """
    
class is_first_step_of(BaseProperty):
    """
    A relation stating that some task is the first one in a workflow.
    A relation stating that some task is the first one in a workflow.
    """
    
class has_friction_attribute(BaseProperty):
    """
    A relation between physical objects and their friction attribute.
    A relation between physical objects and their friction attribute.
    """
    
class has_goal(BaseProperty):
    """
    A relation from an Entity to a Goal it pursues. Agents can pursue Goals, and Tasks are also construed as pursuing Goals.
    """
    
class has_initial_scene(BaseProperty):
    """
    A relation between StateTransitions and Scenes, which identifies the scene the transition starts from.
    """
    
class has_initial_situation(BaseProperty):
    """
    A relation between SituationTransitions and Situations, which identifies the Situation the transition starts from.
    A relation between SituationTransitions and Situations, which identifies the Situation the transition starts from.
    """
    
class has_initial_state(BaseProperty):
    """
    A relation which connects a Transition to the Situation it starts from.
    """
    
class is_initial_situation_of(BaseProperty):
    """
    A relation between SituationTransitions and Situations, which identifies the Situation the transition starts from.
    A relation between SituationTransitions and Situations, which identifies the Situation the transition starts from.
    """
    
class has_input_parameter(BaseProperty):
    """
    A relation between a Task and one of its input parameters.
    A relation from an EventType (typically, a Task) and a parameter describing some state of affairs before the event classified by the EventType takes place, and which contributes towards that event happening.
    """
    
class has_joint_limit(BaseProperty):
    """
    Relates a joint to its physical limits.
    Relates a joint to its physical limits.
    """
    
class is_joint_limit_of(BaseProperty):
    """
    Relates a joint to its physical limits.
    Relates a joint to its physical limits.
    """
    
class has_joint_state(BaseProperty):
    """
    Relates a joint to its state.
    Relates a joint to its state.
    """
    
class is_joint_state_of(BaseProperty):
    """
    Relates a joint to its state.
    Relates a joint to its state.
    """
    
class has_component(BaseProperty):
    """
    The hasProperPart relation without transitivity, holding between an Object (the system) and another (the component), and assuming a Design that structures the Object.
    """
    
class is_link_of(BaseProperty):
    """
    Relates an object to its kinematic components.
    Relates an object to its kinematic components.
    """
    
class has_localization(BaseProperty):
    """
    Relates an object to its localization quality.
    Relates an object to its localization quality.
    """
    
class is_localization_of(BaseProperty):
    """
    Relates a localization quality to the object the localization belongs to.
    Relates a localization quality to the object the localization belongs to.
    """
    
class has_mass_attribute(BaseProperty):
    """
    A relation between physical objects and their mass.
    A relation between physical objects and their mass.
    """
    
class is_mass_attribute_of(BaseProperty):
    """
    A relation between physical objects and their mass.
    A relation between physical objects and their mass.
    """
    
class has_net_force(BaseProperty):
    """
    A relation between a physical object and the total force acting on it.
    A relation between a physical object and the total force acting on it.
    """
    
class is_net_force_of(BaseProperty):
    """
    A relation between a physical object and the total force acting on it.
    A relation between a physical object and the total force acting on it.
    """
    
class has_next_step(BaseProperty):
    """
    An ordering relation between tasks in a workflow, saying that a task is followed by another.
    An ordering relation between tasks in a workflow, saying that a task is followed by another.
    """
    
class directly_precedes(BaseProperty):
    """
    The intransitive precedes relation. For example, Monday directly precedes Tuesday. Directness of precedence depends on the designer conceptualization.
    """
    
class has_previous_step(BaseProperty):
    """
    An ordering relation between tasks in some workflow, stating that a task is preceded by another.
    An ordering relation between tasks in some workflow, stating that a task is preceded by another.
    """
    
class has_output_parameter(BaseProperty):
    """
    A relation between a Task and one of its output parameters.
    A relation from an EventType (typically a Task) to a Parameter describing an outcome of the event classified by the EventType.
    """
    
class has_parent_link(BaseProperty):
    """
    Relates a joint to the link it connects which is closer to the root of the kinematic chain.
    Relates a joint to the link it connects which is closer to the root of the kinematic chain.
    """
    
class is_parent_link_of(BaseProperty):
    """
    Relates a joint to the link it connects which is closer to the root of the kinematic chain.
    Relates a joint to the link it connects which is closer to the root of the kinematic chain.
    """
    
class has_phase(BaseProperty):
    """
    A relation used to describe the structure of an Action or Process in terms of phases, i.e. subprocesses and states that occur during its unfolding.
    """
    
class has_predecessor(BaseProperty):
    """
    Indicates that a Task is the predecessor in a Succedence Relation; that is, this is the task to execute first.
    Indicates that a Task is the predecessor in a Succedence Relation; that is, this is the task to execute first.
    """
    
class has_preference(BaseProperty):
    ...
    
class is_preference_of(BaseProperty):
    """
    Relates a preference quality to the agent the preference belongs to.
    Relates a preference quality to the agent the preference belongs to.
    """
    
class directly_follows(BaseProperty):
    """
    The intransitive follows relation. For example, Wednesday directly precedes Thursday. Directness of precedence depends on the designer conceptualization.
    """
    
class has_process_type(BaseProperty):
    """
    A relation between roles and process types, e.g. a catalysator is needed to trigger some chemical reaction.
    A relation between roles and process types, e.g. a catalysator is needed to trigger some chemical reaction.
    """
    
class is_related_to_concept(BaseProperty):
    """
    Any relation between concepts, e.g. superordinated, conceptual parthood, having a parameter, having a task, superordination, etc.
    """
    
class is_process_type_of(BaseProperty):
    """
    A relation between roles and process types, e.g. a catalysator is needed to trigger some chemical reaction.
    A relation between roles and process types, e.g. a catalysator is needed to trigger some chemical reaction.
    """
    
class has_quale(BaseProperty):
    """
    Relates a quality to its "value", called quale, which is an atomic quality region.
    Relates a quality to its "value", called quale, which is an atomic quality region.
    """
    
class is_quale_of(BaseProperty):
    """
    Relates a quality to its "value", called quale, which is an atomic quality region.
    Relates a quality to its "value", called quale, which is an atomic quality region.
    """
    
class has_root_link(BaseProperty):
    """
    Relates an object to kinematic components at the root of the kinematic chain.
    Relates an object to kinematic components at the root of the kinematic chain.
    """
    
class is_root_link_of(BaseProperty):
    """
    Relates an object to kinematic components at the root of the kinematic chain.
    Relates an object to kinematic components at the root of the kinematic chain.
    """
    
class has_shape(BaseProperty):
    """
    Relates an object to its shape quality.
    Relates an object to its shape quality.
    """
    
class is_shape_of(BaseProperty):
    """
    Relates a shape quality to the object the shape belongs to.
    Relates a shape quality to the object the shape belongs to.
    """
    
class has_shape_region(BaseProperty):
    """
    A relation between physical objects and their shape attribute.
    A relation between physical objects and their shape attribute.
    """
    
class is_shape_region_of(BaseProperty):
    """
    Relates a shape to a physical object that has it.
    Relates a shape to a physical object that has it.
    """
    
class has_software_agent(BaseProperty):
    """
    A relation from an Event and the SoftwareAgent responsible for making that Event happen.
    """
    
class has_space_region(BaseProperty):
    """
    Relates an entity to a space region.
    Relates an entity to a space region.
    """
    
class is_space_region_for(BaseProperty):
    """
    Relates a space region to an entity.
    Relates a space region to an entity.
    """
    
class has_state_type(BaseProperty):
    """
    A relation between roles and state types, e.g. 'the chair is the supporter of the person sitting on it'.
    A relation between roles and state types, e.g. 'the chair is the supporter of the person sitting on it'.
    """
    
class is_state_type_of(BaseProperty):
    """
    A relation between roles and state types, e.g. 'the chair is the supporter of the person sitting on it'.
    A relation between roles and state types, e.g. 'the chair is the supporter of the person sitting on it'.
    """
    
class has_status(BaseProperty):
    """
    A relation from an Entity to a Quality that is indicative of the Entity's state, e.g. if it is a device, its state of operation.
    """
    
class is_step_of(BaseProperty):
    """
    A relation stating that a task is a step in a workflow.
    A relation stating that a task is a step in a workflow.
    """
    
class has_succedence(BaseProperty):
    """
    A relation between a Workflow and a Succedence that appears in it.
    A relation between a Workflow and a Succedence that appears in it.
    """
    
class has_successor(BaseProperty):
    """
    Indicates that a Task is the successor in a Succedence Relation: that is, it is the Task to execute last.
    Indicates that a Task is the successor in a Succedence Relation: that is, it is the Task to execute last.
    """
    
class has_task(BaseProperty):
    """
    A relation to indicate that a Task is part of a Workflow or ordering Relation: that is, the task may be executed during the execution of the Workflow, or there exists some Relation between the Tasks that informs how their executions are to be located in time.
    A relation to indicate that a Task is part of a Workflow or ordering Relation: that is, the task may be executed during the execution of the Workflow, or there exists some Relation between the Tasks that informs how their executions are to be located in time.
    """
    
class has_terminal_situation(BaseProperty):
    """
    A relation between SituationTransitions and Situations, which identifies the Situation the transition ends at.
    A relation between SituationTransitions and Situations, which identifies the Situation the transition ends at.
    """
    
class has_terminal_state(BaseProperty):
    """
    A relation from a Transition to the Situation it ends in.
    """
    
class is_terminal_situation_of(BaseProperty):
    """
    A relation between SituationTransitions and Situations, which identifies the Situation the transition ends at.
    A relation between SituationTransitions and Situations, which identifies the Situation the transition ends at.
    """
    
class includes_concept(BaseProperty):
    """
    A relation recording that a Situation has a Concept as participant in some sort of role.
    """
    
class includes_record(BaseProperty):
    """
    A relationship indicating that an Event has been recorded by an InformationObject
    """
    
class includes_situation(BaseProperty):
    """
    A relation recording that a Situation has a (sub) Situation as participant in some role.
    """
    
class involves_artifact(BaseProperty):
    """
    Artifact participation.
    """
    
class involves_effector(BaseProperty):
    """
    Effector participation.
    Effector participation.
    """
    
class has_participant(BaseProperty):
    """
    A relation between an object and a process, e.g. 'John took part in the discussion', 'a large mass of snow fell during the avalanche', or 'a cook, some sugar, flour, etc. are all present in the cooking of a cake'.
    """
    
class is_effector_involved_in(BaseProperty):
    """
    Effector participation.
    Effector participation.
    """
    
class involves_place(BaseProperty):
    """
    A relation recording that an Event makes some use of a PhysicalPlace. Typically this place is where the Event is located.
    """
    
class follows(BaseProperty):
    """
    A relation between entities, expressing a 'sequence' schema. 
    E.g. 'year 2000 follows 1999', 'preparing coffee' follows 'deciding what coffee to use', 'II World War follows I World War', etc. 
    It can be used between tasks, processes or time intervals, and subproperties would fit best in order to distinguish the different uses.
    """
    
class is_region_for(BaseProperty):
    """
    A relation between entities and regions, e.g. 'the color of my car is red'.
    """
    
class is_answered_by(BaseProperty):
    """
    A relation from a Query to an Agent who answers it.
    A relation from a Query to an Agent who answers it.
    """
    
class involves_agent(BaseProperty):
    """
    Agent participation.
    """
    
class is_artifact_involved_in(BaseProperty):
    """
    Artifact participation.
    """
    
class is_asked_by(BaseProperty):
    """
    A relation from a Query to the Agent who asks it.
    A relation from a Query to the Agent who asks it.
    """
    
class is_role_defined_in(BaseProperty):
    """
    A relation between a description and a role, e.g. the role 'Ingredient' is defined in the recipe for a cake.
    """
    
class is_caused_by_event(BaseProperty):
    """
    A relation recording that a Transition is the result of some Event.
    """
    
class is_constituent_of(BaseProperty):
    """
    'Constituency' depends on some layering of  the world described by the ontology. For example, scientific granularities (e.g. body-organ-tissue-cell) or ontological 'strata' (e.g. social-mental-biological-physical) are  typical layerings. 
    Intuitively, a constituent is a part belonging to a lower layer. Since layering is actually a partition of the world described by the ontology, constituents are not properly classified as parts, although this kinship can be intuitive for common sense.
    A desirable advantage of this distinction is that we are able to talk e.g. of physical constituents of non-physical objects (e.g. systems), while this is not possible in terms of parts.
    Example of are the persons constituting a social system, the molecules constituting a person, the atoms constituting a river, etc. 
    In all these examples, we notice a typical discontinuity between the constituted and the constituent object: e.g. a social system is conceptualized at a different layer from the persons that constitute it, a person is conceptualized at a different layer from the molecules that constitute them, and a river is conceptualized at a different layer from the atoms that constitute it.
    """
    
class is_quality_of(BaseProperty):
    """
    A relation between entities and qualities, e.g. 'Dmitri's skin is yellowish'.
    """
    
class is_concept_included_in(BaseProperty):
    """
    A relation recording that a Concept participates in a Situation in some way.
    """
    
class is_contained_in(BaseProperty):
    """
    The inverse of the contains relation. See the contains relation for details.
    """
    
class is_created_output_of(BaseProperty):
    """
    A relation between a created output role and its Task. The difference to isOutputRoleOf is that the latter is also applicable, e.g., for Deciding between objects, where the selected object is not created, but still an outcome of that task.
    A relation between a created output role and its Task. The difference to isOutputRoleOf is that the latter is also applicable, e.g., for Deciding between objects, where the selected object is not created, but still an outcome of that task.
    """
    
class is_output_role_of(BaseProperty):
    """
    A relation between an output roles and its Task.
    """
    
class is_task_of_created_role(BaseProperty):
    """
    A relation between a Task and one of its output roles. The difference to IsTaskOfOutputRole is that the latter is also applicable, e.g., for Deciding between objects, where the selected object is not created, but still an outcome of that task.
    A relation between a Task and one of its output roles. The difference to IsTaskOfOutputRole is that the latter is also applicable, e.g., for Deciding between objects, where the selected object is not created, but still an outcome of that task.
    """
    
class is_defined_in(BaseProperty):
    """
    A relation between a Description and a Concept, e.g. a Workflow for a governmental Organization defines the Role 'officer', or 'the Italian Traffic Law defines the role Vehicle'.
    """
    
class is_deposit_of(BaseProperty):
    """
    A spatial relation holding between an object (the deposit), and objects that are located ontop of it.
    A spatial relation holding between an object (the deposit), and objects that are located ontop of it.
    """
    
class is_ontop_of(BaseProperty):
    """
    A spatial relation holding between an object (the deposit), and objects that are located ontop of it.
    A spatial relation holding between an object (the deposit), and objects that are located ontop of it.
    """
    
class is_design_for(BaseProperty):
    """
    A special relation between a Design and an Object, to indicate that the Design describes a way to construct the Object.
    A special relation between a Design and an Object, to indicate that the Design describes a way to construct the Object.
    """
    
class is_designed_by(BaseProperty):
    """
    A special relation between a Design and an Object, to indicate that the Object is described by the Design.
    A special relation between a Design and an Object, to indicate that the Object is described by the Design.
    """
    
class is_direct_source_for(BaseProperty):
    """
    The (transitive) relation between an information object and another which was derived from the former.
    """
    
class is_realized_by(BaseProperty):
    """
    A relation between an information realization and an information object, e.g. the paper copy of the Italian Constitution realizes the text of the Constitution.
    """
    
class has_role(BaseProperty):
    """
    A relation between an object and a role, e.g. the person 'John' has role 'student'.
    """
    
class is_input_role_of(BaseProperty):
    """
    A relation between an input roles and its Task.
    """
    
class is_role_of(BaseProperty):
    """
    A relation between an object and a role, e.g. 'student' is the role of 'John'.
    """
    
class realizes(BaseProperty):
    """
    A relation between an information realization and an information object, e.g. the paper copy of the Italian Constitution realizes the text of the Constitution.
    """
    
class is_participant_in(BaseProperty):
    """
    A relation between an object and a process, e.g. 'John took part in the discussion', 'a large mass of snow fell during the avalanche', or 'a cook, some sugar, flour, etc. are all present in the cooking of a cake'.
    """
    
class is_occurring_in(BaseProperty):
    """
    A relation between an event and an event type, e.g. 'taking the cup from the table' is an occurence of the motion 'approaching'.
    """
    
class is_executor_defined_in(BaseProperty):
    ...
    
class is_initial_scene_of(BaseProperty):
    """
    A relation between StateTransitions and Scenes, which identifies the scene the transition starts from.
    """
    
class is_initial_state_of(BaseProperty):
    """
    A relation recording that a Situation was where a Transition began.
    """
    
class is_input_parameter_for(BaseProperty):
    """
    A relation between a Task and one of its input parameters.
    A relation from a Parameter to an EventType (typically, a Task). The parameter describes some state of affairs that precedes and will contribute to the event classified by the EventType.
    """
    
class is_component_of(BaseProperty):
    """
    The asymmetric isProperPartOf relation without transitivity, holding between an Object (the system) and another (the component), and assuming a Design that structures the Object.
    """
    
class is_linked_to(BaseProperty):
    """
    A spatial relation holding between objects that are linked with each other such that they resist spatial separation.
    A spatial relation holding between objects that are linked with each other such that they resist spatial separation.
    """
    
class is_motion_description_for(BaseProperty):
    """
    A special relation between a Motion Plan and a Motion, to indicate that the Motion Plan describes a way to achieve the Motion.
    A special relation between a Motion Plan and a Motion, to indicate that the Motion Plan describes a way to achieve the Motion.
    """
    
class is_moved_by_agent(BaseProperty):
    """
    A relation from an object to an agent who causes it to move.
    A relation from an object to an agent who causes it to move.
    """
    
class moves_object(BaseProperty):
    """
    A relation from an agent to an object that the agent causes to move.
    A relation from an agent to an object that the agent causes to move.
    """
    
class is_ordered_by(BaseProperty):
    """
    The relation between an 'Order item' and the 'Order' that sorts them (via the relations 'precedes' and 'follows')
    The relation between an 'Order item' and the 'Order' that sorts them (via the relations 'precedes' and 'follows')
    """
    
class orders(BaseProperty):
    """
    The relation between an 'Order' and the sorted 'Order item' (sorted via the relations 'precedes' and 'follows' between the 'Order item's)
    The relation between an 'Order' and the sorted 'Order item' (sorted via the relations 'precedes' and 'follows' between the 'Order item's)
    """
    
class is_output_parameter_for(BaseProperty):
    """
    A relation between a Task and one of its output parameters.
    A relation from a Parameter to an EventType (typically, a Task). The parameter describes an outcome of the event classified by the EventType.
    """
    
class is_performed_by(BaseProperty):
    """
    A relation from an Action to the Agent who performs it.
    A relation from an Action to the Agent who performs it.
    """
    
class is_physically_contained_in(BaseProperty):
    """
    A spatial relation holding between an object (the container), and objects it contains.
    A spatial relation holding between an object (the container), and objects it contains.
    """
    
class is_place_involved_in(BaseProperty):
    """
    A relation recording that a PhysicalPlace is involved in some Event; typically, this is where the Event is located.
    """
    
class is_plan_for(BaseProperty):
    """
    A special relation between a Plan and a Task, to indicate that the Plan describes a way to achieve the Task.
    """
    
class is_record_included_by(BaseProperty):
    """
    A relationship indicating that an InformationObject is a recording of an Event.
    """
    
class is_replaced_by(BaseProperty):
    """
    The relation between a State that is replaced by another, e.g., the state of a bowl of fruits containing some objects is replaced by a new containment state when one object is taken away (in this example, we simplified the relation between the State and its type).
    The relation between a State that is replaced by another, e.g., the state of a bowl of fruits containing some objects is replaced by a new containment state when one object is taken away (in this example, we simplified the relation between the State and its type).
    """
    
class replaces(BaseProperty):
    """
    The relation between a State that replaces another, e.g., the state of a bowl of fruits containing some objects is replaced by a new containment state when one object is taken away (in this example, we simplified the relation between the State and its type).
    The relation between a State that replaces another, e.g., the state of a bowl of fruits containing some objects is replaced by a new containment state when one object is taken away (in this example, we simplified the relation between the State and its type).
    """
    
class is_event_included_in(BaseProperty):
    ...
    
class is_situation_included_in(BaseProperty):
    """
    A relation recording that a Situation participates in another in some role, or can be considered as a subsituation of the other.
    """
    
class is_source_for(BaseProperty):
    """
    The (transitive) relation between an information object and another which was derived from the former.
    """
    
class is_task_defined_in(BaseProperty):
    """
    A relation between a description and a task, e.g. the task 'boil' is defined in a recipe for a cake.
    """
    
class is_supported_by(BaseProperty):
    """
    A relation between an object (the supporter) and another object (the supportee) where the supporter cancels the effect of gravity on the supportee.
    Relates a supportee to one of its supporters.
    A relation between an object (the supporter) and another object (the supportee) where the supporter cancels the effect of gravity on the supportee.
    Relates a supportee to one of its supporters.
    """
    
class has_common_boundary(BaseProperty):
    """
    A relation to encode either formal or informal characterizations of 'boundaries' common to two different entities: an Event that ends when another begins, two abstract regions that have a common topological boundary, two objects that are said to be 'in contact' from a commonsense perspective, etc.
    """
    
class supports(BaseProperty):
    """
    A relation between an object (the supporter) and another object (the supportee) where the supporter cancels the effect of gravity on the supportee.
    Relates a supportee to one of its supporters.
    A relation between an object (the supporter) and another object (the supportee) where the supporter cancels the effect of gravity on the supportee.
    Relates a supportee to one of its supporters.
    """
    
class is_task_of_output_role(BaseProperty):
    """
    A relation between a Task and one of its output roles.
    """
    
class is_task_of_input_role(BaseProperty):
    """
    A relation between a Task and one of its input roles.
    """
    
class is_terminal_scene_of(BaseProperty):
    """
    A relation between StateTransitions and Scenes, which identifies the scene the transition is expected to end at.
    """
    
class is_terminal_state_of(BaseProperty):
    """
    A relation recording that a Situation was where a Transition ended.
    """
    
class is_terminated_by(BaseProperty):
    """
    The association between an Event that is terminated by another Event, e.g., the Action of picking an apple from a bowl of fruits terminates the State of containment between the apple and the bowl.
    The association between an Event that is terminated by another Event, e.g., the Action of picking an apple from a bowl of fruits terminates the State of containment between the apple and the bowl.
    """
    
class terminates(BaseProperty):
    """
    The association between an Event that terminates another Event, e.g., the Action of picking an apple from a bowl of fruits terminates the State of containment between the apple and the bowl.
    The association between an Event that terminates another Event, e.g., the Action of picking an apple from a bowl of fruits terminates the State of containment between the apple and the bowl.
    """
    
class meets(BaseProperty):
    """
    A relation between entities, expressing a 'sequence' schema where one of the entities exactly ends where the other entity starts.
    """
    
class met_by(BaseProperty):
    """
    A relation between entities, expressing a 'sequence' schema where one of the entities exactly ends where the other entity starts.
    """
    
class overlapped_by(BaseProperty):
    """
    A schematic relation between any entities that also implies ordering, e.g. "she has worked into the night".
    """
    
class overlapped_on(BaseProperty):
    """
    A schematic relation between any entities that also implies ordering, e.g. "she has worked into the night".
    """
    
class simultaneous(BaseProperty):
    """
    `A simultaneous B` means that A strictly starts and ends at the same time instant as B, i.e. their temporal extend is equal.
    """
    
class started_by(BaseProperty):
    """
    `A starts B` means that A starts exactly where B starts, and that A strictly ends before B. As in "I start my day with a coffee".
    """
    
class starts(BaseProperty):
    """
    `A starts B` means that A starts exactly where B starts, and that A strictly ends before B. As in "I start my day with a coffee".
    """
    
class transitions_back(BaseProperty):
    """
    A property which relates a Transient to an Object it both changes from and changes into. This is useful to model objects which, through participation in a process, transform themselves so that an ontological reclassification is necessary, however this transformation is reversible and at the end of the process the objects revert to their previous kind. An example of this is catalysts in chemistry.
    A property which relates a Transient to an Object it both changes from and changes into. This is useful to model objects which, through participation in a process, transform themselves so that an ontological reclassification is necessary, however this transformation is reversible and at the end of the process the objects revert to their previous kind. An example of this is catalysts in chemistry.
    """
    
class transitions_from(BaseProperty):
    """
    A property which relates a Transient to an Object it changes from. This is useful to model objects which, through participation in a process, transform themselves so that an ontological reclassification is necessary. An example of this is dough undergoing the Maillard reaction through baking.
    A property which relates a Transient to an Object it changes from. This is useful to model objects which, through participation in a process, transform themselves so that an ontological reclassification is necessary. An example of this is dough undergoing the Maillard reaction through baking.
    """
    
class transitions_to(BaseProperty):
    """
    A property which relates a Transient to an Object it changes into. This is useful to model objects which, through participation in a process, transform themselves so that an ontological reclassification is necessary. An example of this is baked dough eventually becoming bread by completing a baking process.
    A property which relates a Transient to an Object it changes into. This is useful to model objects which, through participation in a process, transform themselves so that an ontological reclassification is necessary. An example of this is baked dough eventually becoming bread by completing a baking process.
    """
    
class has_expected_terminal_situation(BaseProperty):
    """
    A relation between a Transition and the Situation it is expected to, and does actually, end at. You can assert this relationship when the observed outcome of a transition matches expectations.
    A relation between a Transition and the Situation it is expected to, and does actually, end at. You can assert this relationship when the observed outcome of a transition matches expectations.
    """
    
class has_postcondition(BaseProperty):
    """
    Direct succession applied to situations. 
    E.g., 'A postcondition of our Plan is to have things settled'.
    This should be taken to mean that the postcondition is the situation expected to follow the current situation. Whether the expectation is met is another issue.
    This should be taken to mean that the postcondition is the situation expected to follow the current situation. Whether the expectation is met is another issue.
    """
    
class has_required_initial_situation(BaseProperty):
    """
    A relationship between a Situation x and another Situation y that precedes it, such that without y manifesting, it would be impossible for x to manifest.
    A relationship between a Situation x and another Situation y that precedes it, such that without y manifesting, it would be impossible for x to manifest.
    """
    
class has_precondition(BaseProperty):
    """
    Direct precedence applied to situations. 
    E.g., 'A precondition to declare war against a foreign country is claiming to find nuclear weapons in it'.
    This should be taken to mean: a precondition is a situation without which the current situation would not be possible.
    This should be taken to mean: a precondition is a situation without which the current situation would not be possible.
    """
    
class manifests_in(BaseProperty):
    """
    A relationship indicating that a Situation is realized in an Event that actually happened.
    A relationship indicating that a Situation is realized in an Event that actually happened.
    """
    
class prevented_by(BaseProperty):
    """
    A relationship indicating a Situation is prevented by another from manifesting.
    A relationship indicating a Situation is prevented by another from manifesting.
    """
    
class has_setting(BaseProperty):
    """
    A relation between entities and situations, e.g. 'this morning I've prepared my coffee with a new fantastic Arabica', i.e.: (an amount of) a new fantastic Arabica hasSetting the preparation of my coffee this morning.
    """
    
class prevents(BaseProperty):
    """
    A relationship indicating that a situation does or would prevent another from manifesting. Useful for reasoning about planning (what to do to avoid some bad outcome) and failure recovery (what aspects of the world state prevent continuing the plan?).
    A relationship indicating that a situation does or would prevent another from manifesting. Useful for reasoning about planning (what to do to avoid some bad outcome) and failure recovery (what aspects of the world state prevent continuing the plan?).
    """
    
class is_setting_for(BaseProperty):
    """
    A relation between situations and entities, e.g. 'this morning I've prepared my coffee with a new fantastic Arabica', i.e.: the preparation of my coffee this morning is the setting for (an amount of) a new fantastic Arabica.
    """
    
class acts_for(BaseProperty):
    """
    The relation holding between any Agent, and a SocialAgent. In principle, a SocialAgent requires at least one PhysicalAgent in order to act, but this dependency can be 'delegated'; e.g. a university can be acted for by a department, which on its turm is acted for by physical agents.
    """
    
class acts_through(BaseProperty):
    """
    The relation holding between a PhysicalAgent and a SocialAgent. In principle, a SocialAgent requires at least one PhysicalAgent in order to act, but this dependency can be 'delegated', e.g. a university can be acted for by a department, which is acted for by physical agents. AKA isActedBy
    """
    
class characterizes(BaseProperty):
    """
    A relation between concepts and collections, where a Concept is said to characterize a Collection; it corresponds to a link between the (reified) intensional and extensional interpretations of a _proper subset of_ a (reified) class. This is different from covers, because it refers to an interpretation the entire reified class.
    E.g. the collection of vintage saxophones is characterized by the Concept 'manufactured by hand', while it gets covered by the Concept 'Saxophone' with the Parameter 'Vintage'.
    """
    
class classifies(BaseProperty):
    """
    A relation between a Concept and an Entity, e.g. the Role 'student' classifies a Person 'John'.
    """
    
class conceptualizes(BaseProperty):
    """
    A relation stating that an Agent is internally representing a SocialObject: situations, descriptions, concepts, etc. E.g., 'John believes in the conspiracy theory'; 'Niels Bohr created the solar-system metaphor for the atomic theory'; 'Jacques assumes all swans are white'; 'the task force members share the attack plan'.
    Conceptualizations can be distinguished into different forms, primarily based on the type of SocialObject that is conceptualized. Descriptions and concepts can be 'assumed', situations can be 'believed' or 'known', plans can be 'adopted', etc. (see ontology: http://www.ontologydesignpatterns.org/ont/dul/Conceptualization.owl.
    """
    
class concretely_expresses(BaseProperty):
    """
    A relation between an InformationRealization and a Description, e.g. 'the printout of the Italian Constitution concretelyExpresses the Italian Constitution'. It should be supplied also with a rule stating that the InformationRealization realizes an InformationObject that expresses the Description
    """
    
class coparticipates_with(BaseProperty):
    """
    A relation between two objects participating in a same Event; e.g., 'Vitas and Jimmy are playing tennis'.
    """
    
class covers(BaseProperty):
    """
    A relation between concepts and collections, where a Concept is said to cover a Collection; it corresponds to a link between the (reified) intensional and extensional interpretations of a (reified) class.
    E.g. the collection of vintage saxophones is covered by the Concept 'Saxophone' with the Parameter 'Vintage'.
    """
    
class executes_task(BaseProperty):
    """
    A relation between an action and a task, e.g. 'putting some water in a pot and putting the pot on a fire until the water starts bubbling' executes the task 'boiling'.
    """
    
class expands(BaseProperty):
    """
    A partial order relation that holds between descriptions. It represents the proper part relation between a description and another description featuring the same properties as the former, with at least one additional one.
    Descriptions can be expanded either by adding other descriptions as parts, or by refining concepts that are used by them. 
    An 'intention' to expand must be present (unless purely formal theories are considered, but even in this case a criterion of relevance is usually active).
    """
    
class expresses(BaseProperty):
    """
    A relation between an InformationObject and a 'meaning', generalized here as a 'SocialObject'. For example: 'A Beehive is a structure in which bees are kept, typically in the form of a dome or box.' (Oxford dictionary)'; 'the term Beehive expresses the concept Beehive in my apiculture ontology'.
    The intuition for 'meaning' is intended to be very broad. A separate, large comment is included for those who want to investigate more on what kind of meaning can be represented in what form.
    This is a large comment field for those who want to investigate the different uses of the 'expresses' relation for modeling different approaches to meaning characterization and modeling.
    For example, in all these cases, some aspect of meaning is involved:
    
    - Beehive means "a structure in which bees are kept, typically in the form of a dome or box." (Oxford dictionary)
    - 'Beehive' is a synonym in noun synset 09218159 "beehive|hive" (WordNet)
    - 'the term Beehive can be interpreted as the fact of 'being a beehive', i.e. a relation that holds for concepts such as Bee, Honey, Hosting, etc.'
    - 'the text of Italian apiculture regulation expresses a rule by which beehives should be kept at least one kilometer away from inhabited areas'
    - 'the term Beehive expresses the concept Beehive'
    - ''Beehive' for apiculturists does not express the same meaning as for, say, fishermen'
    - 'Your meaning of 'Beautiful' does not seem to fit mine'
    - ''Beehive' is formally interpreted as the set of all beehives'
    - 'from the term 'Beehive', we can build a vector space of statistically significant cooccurring terms in the documents that contain it'
    - the lexeme 'Belly' expresses the role 'Body_Part' in the frame 'ObservableBodyParts' (FrameNet)
    
    As the examples suggest, the 'meaning of meaning' is dependent on the background approach/theory that one assumes. One can hardly make a summary of the too many approaches and theories of meaning, therefore this relation is maybe the most controversial and difficult to explain; normally, in such cases it would be better to give up formalizing. 
    However, the usefulness of having a 'semantic abstraction' in modeling information objects is so high (e.g. for the semantic web, interoperability, reengineering, etc.), that we accept this challenging task, although without taking any particular position in the debate. 
    We provide here some examples, which we want to generalize upon when using the 'expresses' relation to model semantic aspects of social reality.
    
    In the most common approach, lexicographers that write dictionaries, glossaries, etc. assume that the meaning of a term is a paraphrase (or 'gloss', or 'definition'). 
    Another approach is provided by concept schemes like thesauri and lexicons, which assume that the meaning of a term is a 'concept', encoded as a 'lemma', 'synset', or 'descriptor'.
    Still another approach is that of psychologists and cognitive scientists, which often assume that the meaning of an information object is a concept encoded in the mind or cognitive system of an agent. 
    A radically different approach is taken by social scientists and semioticians, who usually assume that meanings of an information object are spread across the communication practices in which members of a community use that object.
    Another approach that tackles the distributed nature of meaning is assumed by geometrical models of semantics, which assume that the meaning of an InformationObject (e.g. a word) results from the set of informational contexts (e.g. within texts) in which that object is used similarly.
    The logical approach to meaning is still different, since it assumes that the meaning of e.g. a term is equivalent to the set of individuals that the term can be applied to; for example, the meaning of 'Ali' is e.g. an individual person called Ali, the meaning of 'Airplane' is e.g. the set of airplanes, etc. 
    Finally, an approach taken by structuralist linguistics and frame semantics is that a meaning is the relational context in which an information object can be applied; for example, a meaning of 'Airplane' is situated e.g. in the context ('frame') of passenger airline flights.
    
    These different approaches are not necessarily conflicting, and they mostly talk about different aspects of so-called 'semantics'. They can be summarized and modelled within DOLCE-Ultralite as follows (notice that such list is far from exhaustive):
    
    (1) Informal meaning (as for linguistic or commonsense semantics: a distinction is assumed between (informal) meaning and reference; see isAbout for an alternative pattern on reference)
    	- Paraphrase meaning (as for lexicographic semantics). Here it is modelled as the expresses relation between instances of InformationObject and different instances of InformationObject that act as 'paraphrases'
    	- Conceptual meaning (as for 'concept scheme' semantics). Here it is modelled as the expresses relation between instances of InformationObject and instances of Concept
    	- Relational meaning (as for frame semantics). Here it is modelled as the expresses relation between instances of InformationObject and instances of Description
    	- Cognitive meaning (as for 'psychological' semantics). Here it is modelled as the expresses relation between any instance of InformationObject and any different instance of InformationObject that isRealizedBy a mental, cognitive or neural state (depending on which theory of mind is assumed). Such states can be considered here as instances of Process (occurring in the mind, cognitive system, or neural system of an agent)
    	- Cultural meaning (as for 'social science' semantics). Here it is modelled as the expresses relation between instances of InformationObject and instances of SocialObject (institutions, cultural paradigms, norms, social practices, etc.)
    	- Distributional meaning (as for geometrical models of meaning). Here it is modelled as the expresses relation between any instance of InformationObject and any different instance of InformationObject that isFormallyRepresentedIn some (geometrical) Region (e.g. a vector space)
    
    (2) Formal meaning (as for logic and formal semantics: no distinction is assumed between informal meaning and reference, therefore between 'expresses' and 'isAbout', which can be used interchangeably)
    	- Object-level formal meaning (as in the traditional first-order logic semantics). Here it is modelled as the expresses relation between an instance of InformationObject and an instance of Collection that isGroundingFor (in most cases) a Set; isGroundingFor is defined in the ontology: http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl
    	- Modal formal meaning (as in possible-world semantics). Here it is modelled as the expresses relation between an instance of InformationObject and an instance of Collection that isGroundingFor a Set, and which isPartOf some different instance of Collection that isGroundingFor a PossibleWorld
    
    This is only a first step to provide a framework, in which one can model different aspects of meaning. A more developed ontology should approach the problem of integrating the different uses of 'expresses', so that different theories, resources, methods can interoperate.
    """
    
class is_about(BaseProperty):
    """
    A relation between an information object and an Entity (including information objects). It can be used to talk about entities that are references of proper nouns: the proper noun 'Leonardo da Vinci' isAbout the Person Leonardo da Vinci; as well as to talk about sets of entities that can be described by a common noun: the common noun 'person' isAbout the set of all persons in a domain of discourse, which can be represented in DOLCE-Ultralite as an individual of the class: dul:Collection.
    A specific sentence may use common nouns with either a singular or plural reference, or it can even refer to all possible references (e.g. in a lexicographic definition): all those uses are kinds of aboutness.
    
    The isAbout relation is sometimes considered as reflexive, however this is semiotically inaccurate, because information can be about itself ('de dicto' usage, as in 'John is four character long'), but it is typically about something else ('de re' usage, as in 'John loves Mary').
    If a reflexivity exists in general, it rather concerns its realisation, which is always associated with an event, e.g. an utterance, which makes the information denoting itself, besides its aboutness. This is implemented in DUL with the dul:realizesSelfInformation property, which is used with local reflexivity in the dul:InformationRealization class.
    """
    
class expresses_concept(BaseProperty):
    """
    A relation between an InformationObject and a Concept , e.g. the term "dog" expresses the Concept "dog". For expressing a relational meaning, see the more general object property: expresses
    """
    
class far_from(BaseProperty):
    """
    Generic distance relation between any Entity(s). E.g. Rome is far from Beijing, astronomy is far from necromancy.
    """
    
class has_constraint(BaseProperty):
    """
    A relation between parameters and entities. It allows to assert generic constraints (encoded as parameters), e.g. MinimumAgeForDriving isConstraintFor John (where John is a legal subject under the TrafficLaw).
    The intended semantics (not expressible in OWL) is that a Parameter isParameterFor a Concept that classifies an Entity; moreover, it entails that a Parameter parametrizes a Region that isRegionFor that Entity.
    """
    
class has_member(BaseProperty):
    """
    A relation between collections and entities, e.g. 'my collection of saxophones includes an old Adolphe Sax original alto' (i.e. my collection has member an Adolphe Sax alto).
    """
    
class has_parameter(BaseProperty):
    """
    A Concept can have a Parameter that constrains the attributes that a classified Entity can have in a certain Situation, e.g. a 4WheelDriver Role definedIn the ItalianTrafficLaw has a MinimumAge parameter on the Amount 16.
    """
    
class has_proper_part(BaseProperty):
    """
    Asymmetric (so including irreflexive) parthood.
    """
    
class has_task(BaseProperty):
    """
    A relation between roles and tasks, e.g. 'students have the duty of giving exams' (i.e. the Role 'student' hasTask the Task 'giving exams').
    """
    
class has_time_interval(BaseProperty):
    """
    The generic relation between events and time intervals.
    """
    
class includes_action(BaseProperty):
    """
    A relation between situations and actions, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: the preparation of my coffee this morning included a burning of my fingers).
    """
    
class includes_agent(BaseProperty):
    """
    A relation between situations and persons, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: the preparation of my coffee this morning included me).
    """
    
class includes_object(BaseProperty):
    """
    A relation between situations and objects, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: the preparation of my coffee this morning included me).
    """
    
class includes_part(BaseProperty):
    ...
    
class includes_time(BaseProperty):
    """
    A relation between situations and time intervals, e.g. 'this morning I've prepared my coffee and had my fingers burnt' (i.e.: preparing my coffee was held this morning). A data value attached to the time interval typically complements this modelling pattern.
    """
    
class includes_whole(BaseProperty):
    ...
    
class introduces(BaseProperty):
    """
    A relation between a Description and a SocialAgent, e.g. a Constitutional Charter introduces the SocialAgent 'PresidentOfRepublic'.
    """
    
class is_action_included_in(BaseProperty):
    ...
    
class is_agent_included_in(BaseProperty):
    ...
    
class is_agent_involved_in(BaseProperty):
    """
    Agent participation.
    """
    
class is_characterized_by(BaseProperty):
    ...
    
class is_classified_by(BaseProperty):
    """
    A relation between a Concept and an Entity, e.g. 'John is considered a typical rude man'; your last concert constitutes the achievement of a lifetime; '20-year-old means she's mature enough'.
    """
    
class is_concept_expressed_by(BaseProperty):
    """
    A relation between an InformationObject and a Concept , e.g. the term "dog" expresses the Concept "dog". For expressing a relational meaning, see the more general object property: expresses
    """
    
class is_concept_used_in(BaseProperty):
    """
    A more generic relation holding between a Description and a Concept. In order to be used, a Concept must be previously definedIn another Description
    """
    
class is_conceptualized_by(BaseProperty):
    """
    A relation stating that an Agent is internally representing a Description . E.g., 'John believes in the conspiracy theory'; 'Niels Bohr created a solar-system metaphor for his atomic theory'; 'Jacques assumes all swans are white'; 'the task force shares the attack plan'.
    """
    
class is_concretely_expressed_by(BaseProperty):
    """
    A relation between an InformationRealization and a Description, e.g. 'the printout of the Italian Constitution concretelyExpresses the Italian Constitution'. It should be supplied also with a rule stating that the InformationRealization realizes an InformationObject that expresses the Description
    """
    
class is_constraint_for(BaseProperty):
    """
    A relation between parameters and entities. It allows to assert generic constraints (encoded as parameters), e.g. MinimumAgeForDriving isConstraintFor John (where John is a legal subject under the TrafficLaw).
    The intended semantics (not expressible in OWL) is that a Parameter isConstraintFor and Entity if the Parameter isParameterFor a Concept that classifies that Entity; moreover, it entails that a Parameter parametrizes a Region that isRegionFor that Entity. The use in OWL is therefore a shortcut to annotate what Parameter constrains what Entity
    """
    
class is_covered_by(BaseProperty):
    """
    A relation between concepts and collections, where a Concept is said to cover a Collection; it corresponds to a link between the (reified) intensional and extensional interpretations of a (reified) class.
    E.g. the collection of vintage saxophones is covered by the Concept 'Saxophone' with the Parameter 'Vintage'.
    """
    
class is_executed_in(BaseProperty):
    """
    A relation between an action and a task, e.g. 'putting some water in a pot and putting the pot on a fire until the water starts bubbling' executes the task 'boiling'.
    """
    
class is_expanded_in(BaseProperty):
    """
    A partial order relation that holds between descriptions. It represents the proper part relation between a description and another description featuring the same properties as the former, with at least one additional one.
    Descriptions can be expanded either by adding other descriptions as parts, or by refining concepts that are used by them. 
    An 'intention' to expand must be present (unless purely formal theories are considered, but even in this case a criterion of relevance is usually active).
    """
    
class is_expressed_by(BaseProperty):
    """
    A relation between a dul:SocialObject (the 'meaning') and a dul:InformationObject (the 'expression'). 
    For example: 'A Beehive is a structure in which bees are kept, typically in the form of a dome or box.' (Oxford dictionary)'; 'the term Beehive expresses the concept Beehive in my apiculture ontology'.
    The intuition for 'meaning' is intended to be very broad. A separate, large comment is included in the encoding of 'expresses', for those who want to investigate more on what kind of meaning can be represented in what form.
    """
    
class is_reference_of(BaseProperty):
    """
    A relation between information objects and any Entity (including information objects). It can be used to talk about e.g. entities are references of proper nouns: the proper noun 'Leonardo da Vinci' isAbout the Person Leonardo da Vinci; as well as to talk about sets of entities that can be described by a common noun: the common noun 'person' isAbout the set of all persons in a domain of discourse, which can be represented in DOLCE-Ultralite as an individual of the class: Collection .
    The isReferenceOf relation is irreflexive, differently from its inverse isAbout.
    """
    
class is_introduced_by(BaseProperty):
    """
    A relation between a Description and a SocialAgent, e.g. a Constitutional Charter introduces the SocialAgent 'PresidentOfRepublic'.
    """
    
class is_member_of(BaseProperty):
    """
    A relation between collections and entities, e.g. 'the Night Watch by Rembrandt is in the Rijksmuseum collection'; 'Davide is member of the Pen Club', 'Igor is one the subjects chosen for the experiment'.
    """
    
class is_object_included_in(BaseProperty):
    ...
    
class is_observable_at(BaseProperty):
    """
    A relation to represent a (past, present or future) TimeInterval at which an Entity is observable.
    In order to encode a specific time, a data value should be related to the TimeInterval. 
    An alternative way of representing time is the datatype property: hasIntervalDate
    """
    
class is_parameter_for(BaseProperty):
    """
    A Concept can have a Parameter that constrains the attributes that a classified Entity can have in a certain Situation, e.g. a 4WheelDriver Role definedIn the ItalianTrafficLaw has a MinimumAge parameter on the Amount 16.
    """
    
class is_parametrized_by(BaseProperty):
    """
    The relation between a Parameter, e.g. 'MajorAge', and a Region, e.g. '>17 year'.
    """
    
class is_part_of(BaseProperty):
    """
    A relation between any entities, e.g. 'brain is a part of the human body'. See dul:hasPart for additional documentation.
    """
    
class is_postcondition_of(BaseProperty):
    """
    Direct succession applied to situations. 
    E.g., 'Taking some rest is a postcondition of my search for a hotel'.
    """
    
class is_precondition_of(BaseProperty):
    """
    Direct precedence applied to situations. 
    E.g., 'claiming to find nuclear weapons in a foreign country is a precondition to declare war against it'.
    """
    
class is_propert_part_of(BaseProperty):
    """
    http://www.ontologydesignpatterns.org/ont/dul/DUL.owl
    """
    
class is_reference_of_information_realized_by(BaseProperty):
    """
    The relation between entities and information realizations, e.g. between Italy and a paper copy of the text of the Italian Constitution.
    """
    
class is_related_to_description(BaseProperty):
    """
    Any relation between descriptions.
    """
    
class is_satisfied_by(BaseProperty):
    """
    A relation between a Situation and a Description, e.g. the execution of a Plan satisfies that plan.
    """
    
class is_specialized_by(BaseProperty):
    """
    A partial order relation that holds between social objects. It represents the subsumption relation between e.g. a Concept and another Concept that is broader in extensional interpretation, but narrowe in intensional interpretation.
    E.g. PhDStudent Role specializes Student Role
    """
    
class is_subordinated_to(BaseProperty):
    """
    Direct succession applied to concepts. E.g. the role 'Officer' is subordinated to 'Director'.
    """
    
class is_superordinated_to(BaseProperty):
    """
    Direct precedence applied to concepts. E.g. the role 'Executive' is superordinated to 'DepartmentManager'.
    """
    
class is_task_of(BaseProperty):
    """
    A relation between roles and tasks, e.g. 'students have the duty of giving exams' (i.e. the Role 'student' hasTask the Task 'giving exams').
    """
    
class is_time_included_in(BaseProperty):
    ...
    
class is_time_interval_of(BaseProperty):
    """
    The generic relation between time intervals and events.
    """
    
class is_time_of_observation_of(BaseProperty):
    """
    A relation to represent a (past, present or future) TimeInterval at which an Entity is observable.
    In order to encode a specific time, a data value should be related to the TimeInterval. 
    An alternative way of representing time is the datatype property: hasIntervalDate
    """
    
class is_unified_by(BaseProperty):
    """
    A Collection has a unification criterion, provided by a Description; for example, a community of practice can be unified by a shared theory or interest, e.g. the community that makes research on mirror neurons shares some core knowledge about mirror neurons, which can be represented as a Description MirrorNeuronTheory that unifies the community. There can be several unifying descriptions.
    """
    
class near_to(BaseProperty):
    """
    Generic distance relation between any Entity(s). E.g. Rome is near to Florence, astronomy is near to physics.
    """
    
class overlaps(BaseProperty):
    """
    A schematic relation between any entities, e.g. 'the chest region overlaps with the abdomen region', 'my spoken words overlap with hers', 'the time of my leave overlaps with the time of your arrival', 'fibromyalgia overlaps with other conditions'.
    Subproperties and restrictions can be used to specialize overlaps for objects, events, time intervals, etc.
    """
    
class parametrizes(BaseProperty):
    """
    The relation between a Parameter, e.g. 'MajorAgeLimit', and a Region, e.g. '18_year'.
    For a more data-oriented relation, see hasDataValue
    """
    
class realizes_information_about(BaseProperty):
    """
    The relation between entities and information realizations, e.g. between Italy and a paper copy of the text of the Italian Constitution.
    """
    
class realizes_self_information(BaseProperty):
    """
    This relation is a workaround to enable local reflexivity axioms (Self) working with non-simple properties; in this case, dul:realizesInformation About.
    """
    
class same_setting_as(BaseProperty):
    """
    A relation between two entities participating in a same Situation; e.g., 'Our company provides an antivenom service' (the situation is the service, the two entities are the company and the antivenom).
    """
    
class satisfies(BaseProperty):
    """
    A relation between a Situation and a Description, e.g. the execution of a Plan satisfies that plan.
    """
    
class specializes(BaseProperty):
    """
    A partial order relation that holds between social objects. 
    It mainly represents the subsumption relation between e.g. a Concept or Description and another Concept (resp. Description) that is broader in extensional interpretation, but narrower in intensional interpretation. For example, the role PhDStudent specializes the role Student.
    Another possible use is between a Collection that isCoveredBy a Concept A, and another Collection that isCoveredBy a Concept B that on its turm specializes A. For example, the 70,000 series Selmer Mark VI saxophone Collection specializes the Selmer Mark VI saxophone Collection.
    """
    
class unifies(BaseProperty):
    """
    A Collection has a unification criterion, provided by a Description; for example, a community of practice can be unified by a shared theory or interest, e.g. the community that makes research on mirror neurons shares some core knowledge about mirror neurons, which can be represented as a Description MirrorNeuronTheory that unifies the community. There can be several unifying descriptions.
    """
    
class uses_concept(BaseProperty):
    """
    A generic relation holding between a Description and a Concept. In order to be used, a Concept must be previously definedIn another Description. This last condition cannot be encoded for object properties in OWL.
    """
    
