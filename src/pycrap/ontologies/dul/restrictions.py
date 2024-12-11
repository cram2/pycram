from .dependencies import *
from .classes import *
from .individuals import *



Concept.is_a = [SocialObject, is_defined_in.some(Description), has_part.only(Concept)]

Task.is_a = [EventType, has_part.only(Task), is_executed_in.only(Action), is_task_defined_in.only(Description), is_task_of.only(Role)]

Role.is_a = [Concept, classifies.only(Object), has_part.only(Role)]

Entity.is_a = [Thing]

Event.is_a = [Entity, has_participant.some(Object), has_time_interval.some(TimeInterval), has_constituent.only(Event), has_part.only(Event)]

Transition.is_a = [Situation, includes_event.some(Event), includes_object.some(Object), is_setting_for.some(Process), is_setting_for.some(And([Situation, precedes.some(And([Event, precedes.some(Situation)]))])), includes_time.min(3, TimeInterval), is_setting_for.min(2, Situation)]

PhysicalObject.is_a = [Object, has_part.only(PhysicalObject)]

Description.is_a = [SocialObject]

EventType.is_a = [Concept, classifies.only(Event)]

Parameter.is_a = [Concept, classifies.only(Region), has_part.only(Parameter)]

InformationObject.is_a = [InformationEntity, SocialObject]

Quality.is_a = [Entity, has_region.some(Region), is_quality_of.some(Entity), has_constituent.only(Quality), has_part.only(Quality)]

Collection.is_a = [SocialObject, has_part.only(Collection)]

Action.is_a = [Event, has_participant.some(Agent), executes_task.min(1, Thing)]

Region.is_a = [Abstract, has_constituent.only(Region), has_part.only(Region), overlaps.only(Region), precedes.only(Region)]

Object.is_a = [Entity, has_location.some(Entity), is_participant_in.some(Event), has_constituent.only(Object), has_part.only(Object), is_classified_by.only(Role)]

Workflow.is_a = [Plan, defines_role.some(Role), defines_task.some(Task)]

Goal.is_a = [Description]

Situation.is_a = [Entity, satisfies.some(Description)]

Process.is_a = [Event]

Agent.is_a = [Object]

SpaceRegion.is_a = [Region]

Relation.is_a = [Description]

PhysicalArtifact.is_a = [PhysicalObject, is_described_by.some(Plan)]

PhysicalPlace.is_a = [PhysicalObject]

Design.is_a = [Description]

Plan.is_a = [Description, has_component.some(Goal)]

InformationRealization.is_a = [InformationEntity, Or([Event, PhysicalObject, Quality]), realizes.some(InformationObject), realizes_self_information.has_self()]

TimeInterval.is_a = [Region]

PhysicalAttribute.is_a = [Region, is_region_for.only(PhysicalObject)]

DesignedArtifact.is_a = [PhysicalArtifact, is_described_by.some(Design)]

PhysicalAgent.is_a = [Agent, PhysicalObject]

Diagnosis.is_a = [Description]

SocialObject.is_a = [Object, is_expressed_by.some(InformationObject), has_part.only(SocialObject)]

Configuration.is_a = [Collection]

Substance.is_a = [PhysicalBody]

PhysicalBody.is_a = [PhysicalObject]

Organism.is_a = [BiologicalObject, PhysicalAgent]

FormalEntity.is_a = [Abstract]

SocialRelation.is_a = [Relation]

SocialObjectAttribute.is_a = [Region, is_region_for.only(SocialObject)]

Theory.is_a = [Description, has_component.some(Relation)]

Set.is_a = [FormalEntity]

SocialAgent.is_a = [Agent, SocialObject, acts_through.some(PhysicalAgent)]

Abstract.is_a = [Entity]

Amount.is_a = [Region]

BiologicalObject.is_a = [PhysicalBody]

ChemicalObject.is_a = [PhysicalBody]

Classification.is_a = [TimeIndexedRelation, is_setting_for.some(Concept), is_setting_for.some(Entity), is_setting_for.some(TimeInterval)]

TimeIndexedRelation.is_a = [Situation]
TimeIndexedRelation.equivalent_to = [And([Situation, is_setting_for.some(TimeInterval)])]

Collective.is_a = [Collection, has_member.only(Agent)]

CollectiveAgent.is_a = [SocialAgent, acts_through.some(Agent), is_introduced_by.some(Description)]

Community.is_a = [CollectiveAgent]

Contract.is_a = [Description]

DesignedSubstance.is_a = [DesignedArtifact, FunctionalSubstance]

FunctionalSubstance.is_a = [Substance]

Group.is_a = [CollectiveAgent, is_described_by.some(Plan)]

InformationEntity.is_a = [Entity]

LocalConcept.is_a = [Concept]

Method.is_a = [Description]

Narrative.is_a = [Description]

NaturalPerson.is_a = [Person, PhysicalAgent]

Person.is_a = [Agent]

Norm.is_a = [Description]

ObjectAggregate.is_a = [Thing, And([Object, has_part.some(And([Object, is_member_of.some(Collection)]))])]

Organization.is_a = [SocialAgent]

Parthood.is_a = [TimeIndexedRelation, includes_part.some(Entity), includes_whole.some(Entity)]

Pattern.is_a = [Relation]

Personification.is_a = [SocialAgent]

Place.is_a = [SocialObject, is_location_of.min(1, Thing)]

PlanExecution.is_a = [Situation]
PlanExecution.equivalent_to = [satisfies.some(Plan)]

Project.is_a = [Plan, defines_role.some(Role), defines_task.some(Task)]

Right.is_a = [Description, defines_role.min(2, Thing), defines_task.min(1, Thing)]

SocialPerson.is_a = [Person, SocialAgent, acts_through.exactly(1, Thing)]

SpatioTemporalRegion.is_a = [Region, has_constituent.some(SpaceRegion), has_constituent.some(TimeInterval)]

TypeCollection.is_a = [Collection]

UnitOfMeasure.is_a = [Parameter, parametrizes.some(Region)]

WorkflowExecution.is_a = [Situation]
WorkflowExecution.equivalent_to = [satisfies.some(Workflow)]

has_region_data_value.is_a = [DatatypeProperty, has_data_value]
has_region_data_value.domain = [Region]

has_data_value.is_a = [DatatypeProperty]
has_data_value.domain = [Entity]

has_event_date.is_a = [DatatypeProperty, has_data_value]
has_event_date.domain = [Event]
has_event_date.range = [datetime]

has_interval_date.is_a = [DatatypeProperty, has_region_data_value]
has_interval_date.domain = [TimeInterval]
has_interval_date.range = [datetime]

has_parameter_data_value.is_a = [DatatypeProperty, has_data_value]
has_parameter_data_value.domain = [Parameter]

precedes.is_a = [ObjectProperty, TransitiveProperty, associated_with]
precedes.domain = [Entity]
precedes.range = [Entity]

defines.is_a = [ObjectProperty, uses_concept]
defines.domain = [Description]
defines.range = [Concept]

defines_task.is_a = [ObjectProperty, defines]
defines_task.domain = [Description]
defines_task.range = [Task]

is_described_by.is_a = [ObjectProperty, associated_with]
is_described_by.domain = [Entity]
is_described_by.range = [Description]

associated_with.is_a = [ObjectProperty, SymmetricProperty, TransitiveProperty]
associated_with.domain = [Entity]
associated_with.range = [Entity]

follows.is_a = [ObjectProperty, TransitiveProperty, associated_with]
follows.domain = [Entity]
follows.range = [Entity]

is_event_included_in.is_a = [ObjectProperty, has_setting]
is_event_included_in.domain = [Event]
is_event_included_in.range = [Situation]

overlaps.is_a = [ObjectProperty, SymmetricProperty, associated_with]
overlaps.domain = [Entity]
overlaps.range = [Entity]

is_location_of.is_a = [ObjectProperty, associated_with]
is_location_of.domain = [Entity]
is_location_of.range = [Entity]

defines_role.is_a = [ObjectProperty, defines]
defines_role.domain = [Description]
defines_role.range = [Role]

describes.is_a = [ObjectProperty, associated_with]
describes.domain = [Description]
describes.range = [Entity]

includes_event.is_a = [ObjectProperty, is_setting_for]
includes_event.domain = [Situation]
includes_event.range = [Event]

has_member.is_a = [ObjectProperty, associated_with]
has_member.domain = [Collection]
has_member.range = [Entity]

has_constituent.is_a = [ObjectProperty, associated_with]
has_constituent.domain = [Entity]
has_constituent.range = [Entity]

has_region.is_a = [ObjectProperty, associated_with]
has_region.domain = [Entity]
has_region.range = [Region]

has_part.is_a = [ObjectProperty, TransitiveProperty, ReflexiveProperty, associated_with]
has_part.domain = [Entity]
has_part.range = [Entity]

has_quality.is_a = [ObjectProperty, associated_with]
has_quality.domain = [Entity]
has_quality.range = [Quality]

has_parameter.is_a = [ObjectProperty, is_related_to_concept]
has_parameter.domain = [Concept]
has_parameter.range = [Parameter]

has_component.is_a = [ObjectProperty, AsymmetricProperty, has_proper_part]
has_component.domain = [Entity]
has_component.range = [Entity]

directly_precedes.is_a = [ObjectProperty, precedes]
directly_precedes.domain = [Entity]
directly_precedes.range = [Entity]

directly_follows.is_a = [ObjectProperty, follows]
directly_follows.domain = [Entity]
directly_follows.range = [Entity]

is_related_to_concept.is_a = [ObjectProperty, SymmetricProperty, associated_with]
is_related_to_concept.domain = [Concept]
is_related_to_concept.range = [Concept]

involves_agent.is_a = [ObjectProperty, has_participant]
involves_agent.domain = [Event]
involves_agent.range = [Agent]

includes_object.is_a = [ObjectProperty, is_setting_for]
includes_object.domain = [Situation]
includes_object.range = [Object]

is_reference_of.is_a = [ObjectProperty, associated_with]
is_reference_of.domain = [Entity]
is_reference_of.range = [InformationObject]

is_setting_for.is_a = [ObjectProperty, associated_with]
is_setting_for.domain = [Situation]
is_setting_for.range = [Entity]

has_participant.is_a = [ObjectProperty, associated_with]
has_participant.domain = [Event]
has_participant.range = [Object]

is_region_for.is_a = [ObjectProperty, associated_with]
is_region_for.domain = [Region]
is_region_for.range = [Entity]

is_participant_in.is_a = [ObjectProperty, associated_with]
is_participant_in.domain = [Object]
is_participant_in.range = [Event]

is_role_defined_in.is_a = [ObjectProperty, is_defined_in]
is_role_defined_in.domain = [Role]
is_role_defined_in.range = [Description]

is_constituent_of.is_a = [ObjectProperty, associated_with]
is_constituent_of.domain = [Entity]
is_constituent_of.range = [Entity]

is_quality_of.is_a = [ObjectProperty, associated_with]
is_quality_of.domain = [Quality]
is_quality_of.range = [Entity]

is_object_included_in.is_a = [ObjectProperty, has_setting]
is_object_included_in.domain = [Object]
is_object_included_in.range = [Situation]

is_defined_in.is_a = [ObjectProperty, is_concept_used_in]
is_defined_in.domain = [Concept]
is_defined_in.range = [Description]

is_realized_by.is_a = [ObjectProperty, associated_with]
is_realized_by.domain = [InformationObject]
is_realized_by.range = [InformationRealization]

has_role.is_a = [ObjectProperty, is_classified_by]
has_role.domain = [Object]
has_role.range = [Role]

is_role_of.is_a = [ObjectProperty, classifies]
is_role_of.domain = [Role]
is_role_of.range = [Object]

realizes.is_a = [ObjectProperty, associated_with]
realizes.domain = [InformationRealization]
realizes.range = [InformationObject]

is_parameter_for.is_a = [ObjectProperty, is_related_to_concept]
is_parameter_for.domain = [Parameter]
is_parameter_for.range = [Concept]

has_task.is_a = [ObjectProperty, is_related_to_concept]
has_task.domain = [Role]
has_task.range = [Task]

has_location.is_a = [ObjectProperty, associated_with]
has_location.domain = [Entity]
has_location.range = [Entity]

is_component_of.is_a = [ObjectProperty, AsymmetricProperty, is_propert_part_of]
is_component_of.domain = [Entity]
is_component_of.range = [Entity]

is_classified_by.is_a = [ObjectProperty, associated_with]
is_classified_by.domain = [Entity]
is_classified_by.range = [Concept]

classifies.is_a = [ObjectProperty, associated_with]
classifies.domain = [Concept]
classifies.range = [Entity]

is_about.is_a = [ObjectProperty, associated_with]
is_about.domain = [InformationObject]
is_about.range = [Entity]

has_setting.is_a = [ObjectProperty, associated_with]
has_setting.domain = [Entity]
has_setting.range = [Situation]

is_task_defined_in.is_a = [ObjectProperty, is_defined_in]
is_task_defined_in.domain = [Task]
is_task_defined_in.range = [Description]

has_common_boundary.is_a = [ObjectProperty, SymmetricProperty, associated_with]
has_common_boundary.domain = [Entity]
has_common_boundary.range = [Entity]

is_task_of.is_a = [ObjectProperty, is_related_to_concept]
is_task_of.domain = [Task]
is_task_of.range = [Role]

has_postcondition.is_a = [ObjectProperty, directly_precedes]
has_postcondition.domain = [Or([Event, Situation])]
has_postcondition.range = [Or([Event, Situation])]

has_precondition.is_a = [ObjectProperty, directly_follows]
has_precondition.domain = [Or([Event, Situation])]
has_precondition.range = [Or([Event, Situation])]

acts_for.is_a = [ObjectProperty, associated_with]
acts_for.domain = [Agent]
acts_for.range = [SocialAgent]

executes_task.is_a = [ObjectProperty, is_classified_by]
executes_task.domain = [Action]
executes_task.range = [Task]

expresses.is_a = [ObjectProperty, associated_with]
expresses.domain = [InformationObject]
expresses.range = [SocialObject]

is_executed_in.is_a = [ObjectProperty, classifies]
is_executed_in.domain = [Task]
is_executed_in.range = [Action]

is_expressed_by.is_a = [ObjectProperty, associated_with]
is_expressed_by.domain = [SocialObject]
is_expressed_by.range = [InformationObject]

is_part_of.is_a = [ObjectProperty, TransitiveProperty, ReflexiveProperty, associated_with]
is_part_of.domain = [Entity]
is_part_of.range = [Entity]

satisfies.is_a = [ObjectProperty, associated_with]
satisfies.domain = [Situation]
satisfies.range = [Description]

realizes_self_information.is_a = [ObjectProperty, realizes_information_about]

acts_through.is_a = [ObjectProperty, associated_with]
acts_through.domain = [SocialAgent]
acts_through.range = [Agent]

characterizes.is_a = [ObjectProperty, associated_with]
characterizes.domain = [Concept]
characterizes.range = [Collection]

is_characterized_by.is_a = [ObjectProperty, associated_with]
is_characterized_by.domain = [Collection]
is_characterized_by.range = [Concept]

conceptualizes.is_a = [ObjectProperty, associated_with]
conceptualizes.domain = [Agent]
conceptualizes.range = [SocialObject]

is_conceptualized_by.is_a = [ObjectProperty, associated_with]
is_conceptualized_by.domain = [SocialObject]
is_conceptualized_by.range = [Agent]

concretely_expresses.is_a = [ObjectProperty, associated_with]
concretely_expresses.domain = [InformationRealization]
concretely_expresses.range = [SocialObject]

is_concretely_expressed_by.is_a = [ObjectProperty, associated_with]
is_concretely_expressed_by.domain = [SocialObject]
is_concretely_expressed_by.range = [InformationRealization]

coparticipates_with.is_a = [ObjectProperty, SymmetricProperty, associated_with]
coparticipates_with.domain = [Object]
coparticipates_with.range = [Object]

covers.is_a = [ObjectProperty, associated_with]
covers.domain = [Concept]
covers.range = [Collection]

is_covered_by.is_a = [ObjectProperty, associated_with]
is_covered_by.domain = [Collection]
is_covered_by.range = [Concept]

uses_concept.is_a = [ObjectProperty, associated_with]
uses_concept.domain = [Description]
uses_concept.range = [Concept]

expands.is_a = [ObjectProperty, is_related_to_description]
expands.domain = [Description]
expands.range = [Description]

is_related_to_description.is_a = [ObjectProperty, SymmetricProperty, associated_with]
is_related_to_description.domain = [Description]
is_related_to_description.range = [Description]

is_expanded_in.is_a = [ObjectProperty, is_related_to_description]
is_expanded_in.domain = [Description]
is_expanded_in.range = [Description]

expresses_concept.is_a = [ObjectProperty, expresses]
expresses_concept.domain = [InformationObject]
expresses_concept.range = [Concept]

is_concept_expressed_by.is_a = [ObjectProperty, is_expressed_by]
is_concept_expressed_by.domain = [Concept]
is_concept_expressed_by.range = [InformationObject]

far_from.is_a = [ObjectProperty, SymmetricProperty, associated_with]
far_from.domain = [Entity]
far_from.range = [Entity]

has_proper_part.is_a = [ObjectProperty, TransitiveProperty, has_part]

has_constraint.is_a = [ObjectProperty, is_classified_by]
has_constraint.domain = [Entity]
has_constraint.range = [Parameter]

is_constraint_for.is_a = [ObjectProperty, classifies]
is_constraint_for.domain = [Parameter]
is_constraint_for.range = [Entity]

is_member_of.is_a = [ObjectProperty, associated_with]
is_member_of.domain = [Entity]
is_member_of.range = [Collection]

includes_whole.is_a = [ObjectProperty, is_setting_for]

includes_part.is_a = [ObjectProperty, is_setting_for]

is_postcondition_of.is_a = [ObjectProperty, directly_follows]
is_postcondition_of.domain = [Or([Event, Situation])]
is_postcondition_of.range = [Or([Event, Situation])]

is_precondition_of.is_a = [ObjectProperty, directly_precedes]
is_precondition_of.domain = [Or([Event, Situation])]
is_precondition_of.range = [Or([Event, Situation])]

is_propert_part_of.is_a = [ObjectProperty, TransitiveProperty, is_part_of]

has_time_interval.is_a = [ObjectProperty, has_region]
has_time_interval.domain = [Event]
has_time_interval.range = [TimeInterval]

is_time_interval_of.is_a = [ObjectProperty, is_region_for]
is_time_interval_of.domain = [TimeInterval]
is_time_interval_of.range = [Event]

includes_action.is_a = [ObjectProperty, includes_event]
includes_action.domain = [Situation]
includes_action.range = [Action]

is_action_included_in.is_a = [ObjectProperty, is_event_included_in]
is_action_included_in.domain = [Action]
is_action_included_in.range = [Situation]

includes_agent.is_a = [ObjectProperty, includes_object]
includes_agent.domain = [Situation]
includes_agent.range = [Agent]

is_agent_included_in.is_a = [ObjectProperty, is_object_included_in]
is_agent_included_in.domain = [Agent]
is_agent_included_in.range = [Situation]

includes_time.is_a = [ObjectProperty, is_setting_for]
includes_time.domain = [Situation]
includes_time.range = [TimeInterval]

is_time_included_in.is_a = [ObjectProperty, has_setting]
is_time_included_in.domain = [TimeInterval]
is_time_included_in.range = [Situation]

introduces.is_a = [ObjectProperty, associated_with]
introduces.domain = [Description]
introduces.range = [SocialAgent]

is_introduced_by.is_a = [ObjectProperty, associated_with]
is_introduced_by.domain = [SocialAgent]
is_introduced_by.range = [Description]

is_agent_involved_in.is_a = [ObjectProperty, is_participant_in]
is_agent_involved_in.domain = [Agent]
is_agent_involved_in.range = [Event]

is_concept_used_in.is_a = [ObjectProperty, associated_with]
is_concept_used_in.domain = [Concept]
is_concept_used_in.range = [Description]

is_observable_at.is_a = [ObjectProperty, has_region]
is_observable_at.domain = [Entity]
is_observable_at.range = [TimeInterval]

is_time_of_observation_of.is_a = [ObjectProperty, is_region_for]
is_time_of_observation_of.domain = [TimeInterval]
is_time_of_observation_of.range = [Entity]

is_parametrized_by.is_a = [ObjectProperty, is_classified_by]
is_parametrized_by.domain = [Region]
is_parametrized_by.range = [Parameter]

parametrizes.is_a = [ObjectProperty, classifies]
parametrizes.domain = [Parameter]
parametrizes.range = [Region]

is_reference_of_information_realized_by.is_a = [ObjectProperty, associated_with]
is_reference_of_information_realized_by.domain = [Entity]
is_reference_of_information_realized_by.range = [InformationRealization]

realizes_information_about.is_a = [ObjectProperty, associated_with]
realizes_information_about.domain = [InformationRealization]
realizes_information_about.range = [Entity]

is_satisfied_by.is_a = [ObjectProperty, associated_with]
is_satisfied_by.domain = [Description]
is_satisfied_by.range = [Situation]

is_specialized_by.is_a = [ObjectProperty, TransitiveProperty, associated_with]
is_specialized_by.domain = [SocialObject]
is_specialized_by.range = [SocialObject]

specializes.is_a = [ObjectProperty, TransitiveProperty, associated_with]
specializes.domain = [SocialObject]
specializes.range = [SocialObject]

is_subordinated_to.is_a = [ObjectProperty, directly_follows, is_related_to_concept]
is_subordinated_to.domain = [Concept]
is_subordinated_to.range = [Concept]

is_superordinated_to.is_a = [ObjectProperty, directly_precedes, is_related_to_concept]
is_superordinated_to.domain = [Concept]
is_superordinated_to.range = [Concept]

is_unified_by.is_a = [ObjectProperty, associated_with]
is_unified_by.domain = [Collection]
is_unified_by.range = [Description]

unifies.is_a = [ObjectProperty, associated_with]
unifies.domain = [Description]
unifies.range = [Collection]

near_to.is_a = [ObjectProperty, SymmetricProperty, associated_with]
near_to.domain = [Entity]
near_to.range = [Entity]

same_setting_as.is_a = [ObjectProperty, SymmetricProperty, associated_with]
same_setting_as.domain = [Entity]
same_setting_as.range = [Entity]






