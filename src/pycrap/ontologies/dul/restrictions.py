from .dependencies import *
from .classes import *
from .individuals import *



Concept.is_a = [SocialObject, isDefinedIn.some(Description), hasPart.only(Concept)]

Task.is_a = [EventType, hasPart.only(Task), isExecutedIn.only(Action), isTaskDefinedIn.only(Description), isTaskOf.only(Role)]

Role.is_a = [Concept, classifies.only(Object), hasPart.only(Role)]

Entity.is_a = [Thing]

Event.is_a = [Entity, hasParticipant.some(Object), hasTimeInterval.some(TimeInterval), hasConstituent.only(Event), hasPart.only(Event)]

Transition.is_a = [Situation, includesEvent.some(Event), includesObject.some(Object), isSettingFor.some(Process), isSettingFor.some(Situation & precedes.some(Event & precedes.some(Situation))), includesTime.min(3, TimeInterval), isSettingFor.min(2, Situation)]

PhysicalObject.is_a = [Object, hasPart.only(PhysicalObject)]

Description.is_a = [SocialObject]

EventType.is_a = [Concept, classifies.only(Event)]

Parameter.is_a = [Concept, classifies.only(Region), hasPart.only(Parameter)]

InformationObject.is_a = [InformationEntity, SocialObject]

Quality.is_a = [Entity, hasRegion.some(Region), isQualityOf.some(Entity), hasConstituent.only(Quality), hasPart.only(Quality)]

Collection.is_a = [SocialObject, hasPart.only(Collection)]

Action.is_a = [Event, hasParticipant.some(Agent), executesTask.min(1, Thing)]

Region.is_a = [Abstract, hasConstituent.only(Region), hasPart.only(Region), overlaps.only(Region), precedes.only(Region)]

Object.is_a = [Entity, hasLocation.some(Entity), isParticipantIn.some(Event), hasConstituent.only(Object), hasPart.only(Object), isClassifiedBy.only(Role)]

Workflow.is_a = [Plan, definesRole.some(Role), definesTask.some(Task)]

Goal.is_a = [Description]

Situation.is_a = [Entity, satisfies.some(Description)]

Process.is_a = [Event]

Agent.is_a = [Object]

SpaceRegion.is_a = [Region]

Relation.is_a = [Description]

PhysicalArtifact.is_a = [PhysicalObject, isDescribedBy.some(Plan)]

PhysicalPlace.is_a = [PhysicalObject]

Design.is_a = [Description]

Plan.is_a = [Description, hasComponent.some(Goal)]

InformationRealization.is_a = [InformationEntity, Event | PhysicalObject | Quality, realizes.some(InformationObject), realizesSelfInformation.has_self()]

TimeInterval.is_a = [Region]

PhysicalAttribute.is_a = [Region, isRegionFor.only(PhysicalObject)]

DesignedArtifact.is_a = [PhysicalArtifact, isDescribedBy.some(Design)]

PhysicalAgent.is_a = [Agent, PhysicalObject]

Diagnosis.is_a = [Description]

SocialObject.is_a = [Object, isExpressedBy.some(InformationObject), hasPart.only(SocialObject)]

Configuration.is_a = [Collection]

Substance.is_a = [PhysicalBody]

PhysicalBody.is_a = [PhysicalObject]

Organism.is_a = [BiologicalObject, PhysicalAgent]

FormalEntity.is_a = [Abstract]

SocialRelation.is_a = [Relation]

SocialObjectAttribute.is_a = [Region, isRegionFor.only(SocialObject)]

Theory.is_a = [Description, hasComponent.some(Relation)]

Set.is_a = [FormalEntity]

SocialAgent.is_a = [Agent, SocialObject, actsThrough.some(PhysicalAgent)]

Abstract.is_a = [Entity]

Amount.is_a = [Region]

BiologicalObject.is_a = [PhysicalBody]

ChemicalObject.is_a = [PhysicalBody]

Classification.is_a = [TimeIndexedRelation, isSettingFor.some(Concept), isSettingFor.some(Entity), isSettingFor.some(TimeInterval)]

TimeIndexedRelation.is_a = [Situation]
TimeIndexedRelation.equivalent_to = [Situation & isSettingFor.some(TimeInterval)]

Collective.is_a = [Collection, hasMember.only(Agent)]

CollectiveAgent.is_a = [SocialAgent, actsThrough.some(Agent), isIntroducedBy.some(Description)]

Community.is_a = [CollectiveAgent]

Contract.is_a = [Description]

DesignedSubstance.is_a = [DesignedArtifact, FunctionalSubstance]

FunctionalSubstance.is_a = [Substance]

Group.is_a = [CollectiveAgent, isDescribedBy.some(Plan)]

InformationEntity.is_a = [Entity]

LocalConcept.is_a = [Concept]

Method.is_a = [Description]

Narrative.is_a = [Description]

NaturalPerson.is_a = [Person, PhysicalAgent]

Person.is_a = [Agent]

Norm.is_a = [Description]

ObjectAggregate.is_a = [Thing, Object & hasPart.some(Object & isMemberOf.some(Collection))]

Organization.is_a = [SocialAgent]

Parthood.is_a = [TimeIndexedRelation, includesPart.some(Entity), includesWhole.some(Entity)]

Pattern.is_a = [Relation]

Personification.is_a = [SocialAgent]

Place.is_a = [SocialObject, isLocationOf.min(1, Thing)]

PlanExecution.is_a = [Situation]
PlanExecution.equivalent_to = [satisfies.some(Plan)]

Project.is_a = [Plan, definesRole.some(Role), definesTask.some(Task)]

Right.is_a = [Description, definesRole.min(2, Thing), definesTask.min(1, Thing)]

SocialPerson.is_a = [Person, SocialAgent, actsThrough.exactly(1, Thing)]

SpatioTemporalRegion.is_a = [Region, hasConstituent.some(SpaceRegion), hasConstituent.some(TimeInterval)]

TypeCollection.is_a = [Collection]

UnitOfMeasure.is_a = [Parameter, parametrizes.some(Region)]

WorkflowExecution.is_a = [Situation]
WorkflowExecution.equivalent_to = [satisfies.some(Workflow)]

hasRegionDataValue.is_a = [DatatypeProperty, hasDataValue]
hasRegionDataValue.domain = [Region]

hasDataValue.is_a = [DatatypeProperty]
hasDataValue.domain = [Entity]

hasEventDate.is_a = [DatatypeProperty, hasDataValue]
hasEventDate.domain = [Event]
hasEventDate.range = [<class 'datetime.datetime'>]

hasIntervalDate.is_a = [DatatypeProperty, hasRegionDataValue]
hasIntervalDate.domain = [TimeInterval]
hasIntervalDate.range = [<class 'datetime.datetime'>]

hasParameterDataValue.is_a = [DatatypeProperty, hasDataValue]
hasParameterDataValue.domain = [Parameter]

precedes.is_a = [ObjectProperty, TransitiveProperty, associatedWith]
precedes.domain = [Entity]
precedes.range = [Entity]

defines.is_a = [ObjectProperty, usesConcept]
defines.domain = [Description]
defines.range = [Concept]

definesTask.is_a = [ObjectProperty, defines]
definesTask.domain = [Description]
definesTask.range = [Task]

isDescribedBy.is_a = [ObjectProperty, associatedWith]
isDescribedBy.domain = [Entity]
isDescribedBy.range = [Description]

associatedWith.is_a = [ObjectProperty, SymmetricProperty, TransitiveProperty]
associatedWith.domain = [Entity]
associatedWith.range = [Entity]

follows.is_a = [ObjectProperty, TransitiveProperty, associatedWith]
follows.domain = [Entity]
follows.range = [Entity]

isEventIncludedIn.is_a = [ObjectProperty, hasSetting]
isEventIncludedIn.domain = [Event]
isEventIncludedIn.range = [Situation]

overlaps.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
overlaps.domain = [Entity]
overlaps.range = [Entity]

isLocationOf.is_a = [ObjectProperty, associatedWith]
isLocationOf.domain = [Entity]
isLocationOf.range = [Entity]

definesRole.is_a = [ObjectProperty, defines]
definesRole.domain = [Description]
definesRole.range = [Role]

describes.is_a = [ObjectProperty, associatedWith]
describes.domain = [Description]
describes.range = [Entity]

includesEvent.is_a = [ObjectProperty, isSettingFor]
includesEvent.domain = [Situation]
includesEvent.range = [Event]

hasMember.is_a = [ObjectProperty, associatedWith]
hasMember.domain = [Collection]
hasMember.range = [Entity]

hasConstituent.is_a = [ObjectProperty, associatedWith]
hasConstituent.domain = [Entity]
hasConstituent.range = [Entity]

hasRegion.is_a = [ObjectProperty, associatedWith]
hasRegion.domain = [Entity]
hasRegion.range = [Region]

hasPart.is_a = [ObjectProperty, TransitiveProperty, ReflexiveProperty, associatedWith]
hasPart.domain = [Entity]
hasPart.range = [Entity]

hasQuality.is_a = [ObjectProperty, associatedWith]
hasQuality.domain = [Entity]
hasQuality.range = [Quality]

hasParameter.is_a = [ObjectProperty, isRelatedToConcept]
hasParameter.domain = [Concept]
hasParameter.range = [Parameter]

hasComponent.is_a = [ObjectProperty, AsymmetricProperty, hasProperPart]
hasComponent.domain = [Entity]
hasComponent.range = [Entity]

directlyPrecedes.is_a = [ObjectProperty, precedes]
directlyPrecedes.domain = [Entity]
directlyPrecedes.range = [Entity]

directlyFollows.is_a = [ObjectProperty, follows]
directlyFollows.domain = [Entity]
directlyFollows.range = [Entity]

isRelatedToConcept.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
isRelatedToConcept.domain = [Concept]
isRelatedToConcept.range = [Concept]

involvesAgent.is_a = [ObjectProperty, hasParticipant]
involvesAgent.domain = [Event]
involvesAgent.range = [Agent]

includesObject.is_a = [ObjectProperty, isSettingFor]
includesObject.domain = [Situation]
includesObject.range = [Object]

isReferenceOf.is_a = [ObjectProperty, associatedWith]
isReferenceOf.domain = [Entity]
isReferenceOf.range = [InformationObject]

isSettingFor.is_a = [ObjectProperty, associatedWith]
isSettingFor.domain = [Situation]
isSettingFor.range = [Entity]

hasParticipant.is_a = [ObjectProperty, associatedWith]
hasParticipant.domain = [Event]
hasParticipant.range = [Object]

isRegionFor.is_a = [ObjectProperty, associatedWith]
isRegionFor.domain = [Region]
isRegionFor.range = [Entity]

isParticipantIn.is_a = [ObjectProperty, associatedWith]
isParticipantIn.domain = [Object]
isParticipantIn.range = [Event]

isRoleDefinedIn.is_a = [ObjectProperty, isDefinedIn]
isRoleDefinedIn.domain = [Role]
isRoleDefinedIn.range = [Description]

isConstituentOf.is_a = [ObjectProperty, associatedWith]
isConstituentOf.domain = [Entity]
isConstituentOf.range = [Entity]

isQualityOf.is_a = [ObjectProperty, associatedWith]
isQualityOf.domain = [Quality]
isQualityOf.range = [Entity]

isObjectIncludedIn.is_a = [ObjectProperty, hasSetting]
isObjectIncludedIn.domain = [Object]
isObjectIncludedIn.range = [Situation]

isDefinedIn.is_a = [ObjectProperty, isConceptUsedIn]
isDefinedIn.domain = [Concept]
isDefinedIn.range = [Description]

isRealizedBy.is_a = [ObjectProperty, associatedWith]
isRealizedBy.domain = [InformationObject]
isRealizedBy.range = [InformationRealization]

hasRole.is_a = [ObjectProperty, isClassifiedBy]
hasRole.domain = [Object]
hasRole.range = [Role]

isRoleOf.is_a = [ObjectProperty, classifies]
isRoleOf.domain = [Role]
isRoleOf.range = [Object]

realizes.is_a = [ObjectProperty, associatedWith]
realizes.domain = [InformationRealization]
realizes.range = [InformationObject]

isParameterFor.is_a = [ObjectProperty, isRelatedToConcept]
isParameterFor.domain = [Parameter]
isParameterFor.range = [Concept]

hasTask.is_a = [ObjectProperty, isRelatedToConcept]
hasTask.domain = [Role]
hasTask.range = [Task]

hasLocation.is_a = [ObjectProperty, associatedWith]
hasLocation.domain = [Entity]
hasLocation.range = [Entity]

isComponentOf.is_a = [ObjectProperty, AsymmetricProperty, isPropertPartOf]
isComponentOf.domain = [Entity]
isComponentOf.range = [Entity]

isClassifiedBy.is_a = [ObjectProperty, associatedWith]
isClassifiedBy.domain = [Entity]
isClassifiedBy.range = [Concept]

classifies.is_a = [ObjectProperty, associatedWith]
classifies.domain = [Concept]
classifies.range = [Entity]

isAbout.is_a = [ObjectProperty, associatedWith]
isAbout.domain = [InformationObject]
isAbout.range = [Entity]

hasSetting.is_a = [ObjectProperty, associatedWith]
hasSetting.domain = [Entity]
hasSetting.range = [Situation]

isTaskDefinedIn.is_a = [ObjectProperty, isDefinedIn]
isTaskDefinedIn.domain = [Task]
isTaskDefinedIn.range = [Description]

hasCommonBoundary.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
hasCommonBoundary.domain = [Entity]
hasCommonBoundary.range = [Entity]

isTaskOf.is_a = [ObjectProperty, isRelatedToConcept]
isTaskOf.domain = [Task]
isTaskOf.range = [Role]

hasPostcondition.is_a = [ObjectProperty, directlyPrecedes]
hasPostcondition.domain = [Event | Situation]
hasPostcondition.range = [Event | Situation]

hasPrecondition.is_a = [ObjectProperty, directlyFollows]
hasPrecondition.domain = [Event | Situation]
hasPrecondition.range = [Event | Situation]

actsFor.is_a = [ObjectProperty, associatedWith]
actsFor.domain = [Agent]
actsFor.range = [SocialAgent]

executesTask.is_a = [ObjectProperty, isClassifiedBy]
executesTask.domain = [Action]
executesTask.range = [Task]

expresses.is_a = [ObjectProperty, associatedWith]
expresses.domain = [InformationObject]
expresses.range = [SocialObject]

isExecutedIn.is_a = [ObjectProperty, classifies]
isExecutedIn.domain = [Task]
isExecutedIn.range = [Action]

isExpressedBy.is_a = [ObjectProperty, associatedWith]
isExpressedBy.domain = [SocialObject]
isExpressedBy.range = [InformationObject]

isPartOf.is_a = [ObjectProperty, TransitiveProperty, ReflexiveProperty, associatedWith]
isPartOf.domain = [Entity]
isPartOf.range = [Entity]

satisfies.is_a = [ObjectProperty, associatedWith]
satisfies.domain = [Situation]
satisfies.range = [Description]

realizesSelfInformation.is_a = [ObjectProperty, realizesInformationAbout]

actsThrough.is_a = [ObjectProperty, associatedWith]
actsThrough.domain = [SocialAgent]
actsThrough.range = [Agent]

characterizes.is_a = [ObjectProperty, associatedWith]
characterizes.domain = [Concept]
characterizes.range = [Collection]

isCharacterizedBy.is_a = [ObjectProperty, associatedWith]
isCharacterizedBy.domain = [Collection]
isCharacterizedBy.range = [Concept]

conceptualizes.is_a = [ObjectProperty, associatedWith]
conceptualizes.domain = [Agent]
conceptualizes.range = [SocialObject]

isConceptualizedBy.is_a = [ObjectProperty, associatedWith]
isConceptualizedBy.domain = [SocialObject]
isConceptualizedBy.range = [Agent]

concretelyExpresses.is_a = [ObjectProperty, associatedWith]
concretelyExpresses.domain = [InformationRealization]
concretelyExpresses.range = [SocialObject]

isConcretelyExpressedBy.is_a = [ObjectProperty, associatedWith]
isConcretelyExpressedBy.domain = [SocialObject]
isConcretelyExpressedBy.range = [InformationRealization]

coparticipatesWith.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
coparticipatesWith.domain = [Object]
coparticipatesWith.range = [Object]

covers.is_a = [ObjectProperty, associatedWith]
covers.domain = [Concept]
covers.range = [Collection]

isCoveredBy.is_a = [ObjectProperty, associatedWith]
isCoveredBy.domain = [Collection]
isCoveredBy.range = [Concept]

usesConcept.is_a = [ObjectProperty, associatedWith]
usesConcept.domain = [Description]
usesConcept.range = [Concept]

expands.is_a = [ObjectProperty, isRelatedToDescription]
expands.domain = [Description]
expands.range = [Description]

isRelatedToDescription.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
isRelatedToDescription.domain = [Description]
isRelatedToDescription.range = [Description]

isExpandedIn.is_a = [ObjectProperty, isRelatedToDescription]
isExpandedIn.domain = [Description]
isExpandedIn.range = [Description]

expressesConcept.is_a = [ObjectProperty, expresses]
expressesConcept.domain = [InformationObject]
expressesConcept.range = [Concept]

isConceptExpressedBy.is_a = [ObjectProperty, isExpressedBy]
isConceptExpressedBy.domain = [Concept]
isConceptExpressedBy.range = [InformationObject]

farFrom.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
farFrom.domain = [Entity]
farFrom.range = [Entity]

hasProperPart.is_a = [ObjectProperty, TransitiveProperty, hasPart]

hasConstraint.is_a = [ObjectProperty, isClassifiedBy]
hasConstraint.domain = [Entity]
hasConstraint.range = [Parameter]

isConstraintFor.is_a = [ObjectProperty, classifies]
isConstraintFor.domain = [Parameter]
isConstraintFor.range = [Entity]

isMemberOf.is_a = [ObjectProperty, associatedWith]
isMemberOf.domain = [Entity]
isMemberOf.range = [Collection]

includesWhole.is_a = [ObjectProperty, isSettingFor]

includesPart.is_a = [ObjectProperty, isSettingFor]

isPostconditionOf.is_a = [ObjectProperty, directlyFollows]
isPostconditionOf.domain = [Event | Situation]
isPostconditionOf.range = [Event | Situation]

isPreconditionOf.is_a = [ObjectProperty, directlyPrecedes]
isPreconditionOf.domain = [Event | Situation]
isPreconditionOf.range = [Event | Situation]

isPropertPartOf.is_a = [ObjectProperty, TransitiveProperty, isPartOf]

hasTimeInterval.is_a = [ObjectProperty, hasRegion]
hasTimeInterval.domain = [Event]
hasTimeInterval.range = [TimeInterval]

isTimeIntervalOf.is_a = [ObjectProperty, isRegionFor]
isTimeIntervalOf.domain = [TimeInterval]
isTimeIntervalOf.range = [Event]

includesAction.is_a = [ObjectProperty, includesEvent]
includesAction.domain = [Situation]
includesAction.range = [Action]

isActionIncludedIn.is_a = [ObjectProperty, isEventIncludedIn]
isActionIncludedIn.domain = [Action]
isActionIncludedIn.range = [Situation]

includesAgent.is_a = [ObjectProperty, includesObject]
includesAgent.domain = [Situation]
includesAgent.range = [Agent]

isAgentIncludedIn.is_a = [ObjectProperty, isObjectIncludedIn]
isAgentIncludedIn.domain = [Agent]
isAgentIncludedIn.range = [Situation]

includesTime.is_a = [ObjectProperty, isSettingFor]
includesTime.domain = [Situation]
includesTime.range = [TimeInterval]

isTimeIncludedIn.is_a = [ObjectProperty, hasSetting]
isTimeIncludedIn.domain = [TimeInterval]
isTimeIncludedIn.range = [Situation]

introduces.is_a = [ObjectProperty, associatedWith]
introduces.domain = [Description]
introduces.range = [SocialAgent]

isIntroducedBy.is_a = [ObjectProperty, associatedWith]
isIntroducedBy.domain = [SocialAgent]
isIntroducedBy.range = [Description]

isAgentInvolvedIn.is_a = [ObjectProperty, isParticipantIn]
isAgentInvolvedIn.domain = [Agent]
isAgentInvolvedIn.range = [Event]

isConceptUsedIn.is_a = [ObjectProperty, associatedWith]
isConceptUsedIn.domain = [Concept]
isConceptUsedIn.range = [Description]

isObservableAt.is_a = [ObjectProperty, hasRegion]
isObservableAt.domain = [Entity]
isObservableAt.range = [TimeInterval]

isTimeOfObservationOf.is_a = [ObjectProperty, isRegionFor]
isTimeOfObservationOf.domain = [TimeInterval]
isTimeOfObservationOf.range = [Entity]

isParametrizedBy.is_a = [ObjectProperty, isClassifiedBy]
isParametrizedBy.domain = [Region]
isParametrizedBy.range = [Parameter]

parametrizes.is_a = [ObjectProperty, classifies]
parametrizes.domain = [Parameter]
parametrizes.range = [Region]

isReferenceOfInformationRealizedBy.is_a = [ObjectProperty, associatedWith]
isReferenceOfInformationRealizedBy.domain = [Entity]
isReferenceOfInformationRealizedBy.range = [InformationRealization]

realizesInformationAbout.is_a = [ObjectProperty, associatedWith]
realizesInformationAbout.domain = [InformationRealization]
realizesInformationAbout.range = [Entity]

isSatisfiedBy.is_a = [ObjectProperty, associatedWith]
isSatisfiedBy.domain = [Description]
isSatisfiedBy.range = [Situation]

isSpecializedBy.is_a = [ObjectProperty, TransitiveProperty, associatedWith]
isSpecializedBy.domain = [SocialObject]
isSpecializedBy.range = [SocialObject]

specializes.is_a = [ObjectProperty, TransitiveProperty, associatedWith]
specializes.domain = [SocialObject]
specializes.range = [SocialObject]

isSubordinatedTo.is_a = [ObjectProperty, directlyFollows, isRelatedToConcept]
isSubordinatedTo.domain = [Concept]
isSubordinatedTo.range = [Concept]

isSuperordinatedTo.is_a = [ObjectProperty, directlyPrecedes, isRelatedToConcept]
isSuperordinatedTo.domain = [Concept]
isSuperordinatedTo.range = [Concept]

isUnifiedBy.is_a = [ObjectProperty, associatedWith]
isUnifiedBy.domain = [Collection]
isUnifiedBy.range = [Description]

unifies.is_a = [ObjectProperty, associatedWith]
unifies.domain = [Description]
unifies.range = [Collection]

nearTo.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
nearTo.domain = [Entity]
nearTo.range = [Entity]

sameSettingAs.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
sameSettingAs.domain = [Entity]
sameSettingAs.range = [Entity]






