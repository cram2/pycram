from .dependencies import *
from .classes import *
from .individuals import *


Affordance.is_a = [Relation, describes.only(Disposition), definesBearer.exactly(1, Role), definesTrigger.exactly(1, Role), definesTask.exactly(1, Task)]

Concept.is_a = [SocialObject, isDefinedIn.some(Description), hasPart.only(Concept)]

Task.is_a = [EventType, hasPart.only(Task), isExecutedIn.only(Action), isTaskDefinedIn.only(Description), isTaskOf.only(Role)]

Disposition.is_a = [Extrinsic, isDescribedBy.exactly(1, Affordance), isDescribedBy.exactly(1, Affordance & definesBearer.exactly(1, Role) & definesEventType.exactly(1, EventType) & definesTrigger.exactly(1, Role))]

Role.is_a = [Concept, classifies.only(Object), hasPart.only(Role)]

Setpoint.is_a = [Parameter]

Entity.is_a = [Thing]

Answer.is_a = [Message]

Message.is_a = [Item, hasTask.some(CommunicationTask), classifies.only(InformationRealization)]

Event.is_a = [Entity, hasParticipant.some(Object), hasTimeInterval.some(TimeInterval), hasConstituent.only(Event), hasPart.only(Event)]

Transition.is_a = [Situation, includesEvent.some(Event), includesObject.some(Object), isSettingFor.some(Process), isSettingFor.some(Situation & precedes.some(Event & precedes.some(Situation))), includesTime.min(3, TimeInterval), isSettingFor.min(2, Situation)]

PhysicalObject.is_a = [Object, hasPart.only(PhysicalObject)]

Description.is_a = [SocialObject]

EventType.is_a = [Concept, classifies.only(Event)]

Parameter.is_a = [Concept, classifies.only(Region), hasPart.only(Parameter)]

ProcessType.is_a = [EventType, classifies.only(Process), hasPart.only(ProcessType), isDefinedIn.only(Description)]

InformationObject.is_a = [InformationEntity, SocialObject]

Quality.is_a = [Entity, hasRegion.some(Region), isQualityOf.some(Entity), hasConstituent.only(Quality), hasPart.only(Quality)]

OrderedElement.is_a = [Singleton, follows.only(OrderedElement), precedes.only(OrderedElement), isOrderedBy.exactly(1, Order)]

MotionProcess.is_a = [PhysicsProcess]

Motion.is_a = [ProcessType, isExecutedMotionIn.only(MotionProcess)]

Collection.is_a = [SocialObject, hasPart.only(Collection)]

System.is_a = [SocialObject]

Action.is_a = [Event, hasParticipant.some(Agent), executesTask.min(1, Thing)]

Region.is_a = [Abstract, hasConstituent.only(Region), hasPart.only(Region), overlaps.only(Region), precedes.only(Region)]

Binding.is_a = [Relation, CounterfactualBinding | FactualBinding, RoleFillerBinding | RoleRoleBinding, Inverse(hasBinding).some(Description), hasBindingFiller.exactly(1, Entity), hasBindingRole.exactly(1, Parameter | Role)]

Joint.is_a = [PhysicalBody, hasChildLink.exactly(1, PhysicalObject), hasParentLink.exactly(1, PhysicalObject), hasJointState.max(1, JointState)]
Joint.equivalent_to = [FixedJoint | MovableJoint]

Color.is_a = [Extrinsic, hasRegion.some(ColorRegion), hasRegion.only(ColorRegion)]

Object.is_a = [Entity, hasLocation.some(Entity), isParticipantIn.some(Event), hasConstituent.only(Object), hasPart.only(Object), isClassifiedBy.only(Role)]

ExecutionStateRegion.is_a = [Region]
ExecutionStateRegion.equivalent_to = [OneOf([ExecutionState_Active, ExecutionState_Cancelled, ExecutionState_Failed, ExecutionState_Paused, ExecutionState_Pending, ExecutionState_Succeeded])]

Feature.is_a = [Object, hasPart.only(Feature), isFeatureOf.exactly(1, PhysicalObject)]

Workflow.is_a = [Plan, definesRole.some(Role), definesTask.some(Task)]

FrictionAttribute.is_a = [PhysicalAttribute, hasFrictionValue.exactly(1, float)]

Goal.is_a = [Description]

StateTransition.is_a = [Transition, hasInitialScene.some(Scene), hasTerminalScene.some(Scene), includesEvent.some(Action), satisfies.some(ImageSchemaTheory)]

Scene.is_a = [Situation, includesEvent.some(State), satisfies.some(ImageSchemaTheory)]

SituationTransition.is_a = [Transition]

Situation.is_a = [Entity, satisfies.some(Description)]

NonmanifestedSituation.is_a = [Situation]
NonmanifestedSituation.equivalent_to = [Situation & manifestsIn.exactly(0, Event)]

JointLimit.is_a = [PhysicalAttribute]

JointState.is_a = [PhysicalAttribute, hasJointPosition.exactly(1, float), hasJointVelocity.exactly(1, float), hasJointEffort.max(1, float)]

Localization.is_a = [Extrinsic, hasRegion.some(SpaceRegion), hasRegion.only(SpaceRegion)]

MassAttribute.is_a = [PhysicalAttribute, hasMassValue.exactly(1, float)]

NetForce.is_a = [ForceAttribute]

Process.is_a = [Event]

State.is_a = [Event]

Succedence.is_a = [Relation, Inverse(hasSuccedence).some(Description), hasPredecessor.exactly(1, TaskInvocation), hasSuccessor.exactly(1, TaskInvocation)]

Agent.is_a = [Object]

Preference.is_a = [SocialQuality, isPreferenceOf.only(Agent)]

Shape.is_a = [Intrinsic, hasRegion.some(ShapeRegion), hasRegion.only(ShapeRegion)]

ShapeRegion.is_a = [Region, hasSpaceRegion.max(1, SixDPose)]

SoftwareInstance.is_a = [Thing, SocialAgent & actsFor.some(Agent), isDesignedBy.some(Software)]

SpaceRegion.is_a = [Region]

StateType.is_a = [EventType, classifies.only(State), hasPart.only(StateType)]

Relation.is_a = [Description]

PhysicalArtifact.is_a = [PhysicalObject, isDescribedBy.some(Plan)]

PhysicalEffector.is_a = [FunctionalPart, Inverse(hasPart).some(PhysicalAgent)]

PhysicalPlace.is_a = [PhysicalObject]

QueryingTask.is_a = [IllocutionaryTask, isTaskOf.some(Query), classifies.only(hasParticipant.some(InterrogativeClause))]

Design.is_a = [Description]

MotionDescription.is_a = [ProcessFlow]

Order.is_a = [FormalEntity, orders.some(OrderedElement)]

Plan.is_a = [Description, hasComponent.some(Goal)]

Transient.is_a = [Object, transitionsFrom.some(Object)]

ColorRegion.is_a = [PhysicalAttribute, isRegionFor.only(Color)]

InformationRealization.is_a = [InformationEntity, Event | PhysicalObject | Quality, realizes.some(InformationObject), realizesSelfInformation.has_self()]

ForceAttribute.is_a = [PhysicalAttribute, hasForceValue.exactly(1, float)]

TimeInterval.is_a = [Region]

PhysicalAttribute.is_a = [Region, isRegionFor.only(PhysicalObject)]

API_Specification.is_a = [InterfaceSpecification]

InterfaceSpecification.is_a = [Design]

AbductiveReasoning.is_a = [Reasoning]

Reasoning.is_a = [DerivingInformation]

Accessor.is_a = [Instrument]

Instrument.is_a = [ResourceRole]

Accident.is_a = [Event]

ActionExecutionPlan.is_a = [Plan, definesTask.only(hasParameter.some(Status))]

Status.is_a = [Parameter]

Actuating.is_a = [PhysicalTask]

PhysicalTask.is_a = [Task, classifies.only(Action & (hasParticipant.some(PhysicalAgent) | hasParticipant.some(PhysicalObject)))]

AestheticDesign.is_a = [Design]

AgentRole.is_a = [CausativeRole, classifies.only(Agent)]

CausativeRole.is_a = [EventAdjacentRole]

Agonist.is_a = [Patient]

Patient.is_a = [EventAdjacentRole]

Algorithm.is_a = [Plan]

Alteration.is_a = [ProcessType]

AlterativeInteraction.is_a = [ForceInteraction]

ForceInteraction.is_a = [ProcessType, isProcessTypeOf.some(Agonist), isProcessTypeOf.some(Antagonist)]
ForceInteraction.equivalent_to = [AlterativeInteraction | PreservativeInteraction]

PreservativeInteraction.is_a = [ForceInteraction]

AlteredObject.is_a = [Patient]

Amateurish.is_a = [DexterityDiagnosis]

DexterityDiagnosis.is_a = [BehavioralDiagnosis]

Masterful.is_a = [DexterityDiagnosis]

AnsweringTask.is_a = [Thing]
AnsweringTask.equivalent_to = [IllocutionaryTask & classifies.only(isReactionTo.only(isClassifiedBy.some(CommandingTask))), isTaskOf.some(Answer)]

CommandingTask.is_a = [IllocutionaryTask, classifies.only(hasParticipant.some(ImperativeClause))]

IllocutionaryTask.is_a = [Thing, CommunicationTask & classifies.only(hasParticipant.only(Agent | SocialObject))]

Antagonist.is_a = [Patient]

Appliance.is_a = [DesignedArtifact]

DesignedArtifact.is_a = [PhysicalArtifact, isDescribedBy.some(Design)]

Approaching.is_a = [Locomotion]

Locomotion.is_a = [BodyMovement, DirectedMotion]

ArchiveFile.is_a = [Digital_File]
ArchiveFile.equivalent_to = [Digital_File & realizes.some(ArchiveText)]

ArchiveText.is_a = [Thing]
ArchiveText.equivalent_to = [Structured_Text & expresses.some(FileConfiguration), isGivenMeaningBy.some(ArchiveFormat)]

Digital_File.is_a = [InformationRealization, realizes.some(Structured_Text), hasNameString.some(str)]

ArchiveFormat.is_a = [File_format, givesMeaningTo.only(ArchiveText)]

File_format.is_a = [Computer_Language]

FileConfiguration.is_a = [Thing]
FileConfiguration.equivalent_to = [Configuration & hasMember.only(Digital_File)]

Structured_Text.is_a = [Thing]
Structured_Text.equivalent_to = [Text & isGivenMeaningBy.some(FormalLanguage)]

AreaSurveying.is_a = [Perceiving]

Perceiving.is_a = [Thing, InformationAcquisition & PhysicalTask]

Arm.is_a = [Limb]

Limb.is_a = [PhysicalEffector]
Limb.equivalent_to = [Arm | Leg]

Arranging.is_a = [Constructing]

Constructing.is_a = [PhysicalTask]

ArtificialAgent.is_a = [PhysicalAgent]

PhysicalAgent.is_a = [Agent, PhysicalObject]

Assembling.is_a = [Constructing]

AssertionTask.is_a = [IllocutionaryTask, classifies.only(hasParticipant.some(DeclarativeClause))]

DeclarativeClause.is_a = [ClausalObject]

AssumingArmPose.is_a = [AssumingPose]

AssumingPose.is_a = [PhysicalTask]

AttentionShift.is_a = [MentalTask]

MentalTask.is_a = [Task, classifies.only(MentalAction)]

AvoidedObject.is_a = [Patient]

Avoiding.is_a = [Navigating]

Navigating.is_a = [PhysicalTask]

Barrier.is_a = [Restrictor]

Restrictor.is_a = [Instrument]

BehavioralDiagnosis.is_a = [Diagnosis, hasConstituent.some(Goal)]

Diagnosis.is_a = [Description]

BeneficiaryRole.is_a = [GoalRole]

GoalRole.is_a = [EventAdjacentRole]

CounterfactualBinding.is_a = [Binding]

FactualBinding.is_a = [Binding]

RoleFillerBinding.is_a = [Binding]

RoleRoleBinding.is_a = [Binding, hasBindingFiller.only(Parameter | Role)]

Blockage.is_a = [Disposition, affordsBearer.only(Barrier), affordsTrigger.only(BlockedObject)]

BlockedObject.is_a = [Patient]

BodyMovement.is_a = [Motion, classifies.only(hasParticipant.some(PhysicalAgent) | hasParticipant.some(PhysicalObject & isPartOf.some(PhysicalAgent)))]

Boiling.is_a = [Vaporizing]

Vaporizing.is_a = [PhaseTransition, isProcessTypeOf.some(AlteredObject & classifies.only(Substance))]

BoxShape.is_a = [ShapeRegion, hasHeight.exactly(1, float), hasLength.exactly(1, float), hasWidth.exactly(1, float)]

CanCut.is_a = [Variability, affordsBearer.only(Cutter), affordsTrigger.only(CutObject)]

Variability.is_a = [Disposition, affordsBearer.only(Tool), affordsTrigger.only(AlteredObject)]

Cutter.is_a = [Tool]

CutObject.is_a = [AlteredObject]

Capability.is_a = [Disposition, isDescribedBy.exactly(1, Affordance & definesBearer.exactly(1, Role) & definesTrigger.exactly(1, Role) & definesTask.exactly(1, Task))]

Capacity.is_a = [Intrinsic]

Intrinsic.is_a = [PhysicalQuality]

Catching.is_a = [Actuating]

PickingUp.is_a = [Manipulating]

CausalEventRole.is_a = [CausativeRole]

EventAdjacentRole.is_a = [Role]

CausedMotionTheory.is_a = [ImageSchemaTheory, defines.some(CausalEventRole), defines.some(Instrument), defines.some(Patient), defines.some(PerformerRole), hasPart.some(SourcePathGoalTheory)]

ImageSchemaTheory.is_a = [SchematicTheory]

PerformerRole.is_a = [CausativeRole, classifies.only(Agent)]

SourcePathGoalTheory.is_a = [ImageSchemaTheory, defines.some(Destination), defines.some(Origin), defines.some(PathRole)]

Channel.is_a = [PathRole, hasTask.some(CommunicationTask)]

PathRole.is_a = [SpatioTemporalRole]

CommunicationTask.is_a = [Task, isTaskOf.some(Channel) & isTaskOf.some(Message) & isTaskOf.some(Receiver) & isTaskOf.some(Sender), classifies.only(hasParticipant.only(Agent | SocialObject))]

CheckingObjectPresence.is_a = [Perceiving]

ChemicalProcess.is_a = [Process]

Choice.is_a = [ResultRole]

ResultRole.is_a = [GoalRole]

CircularCylinder.is_a = [CylinderShape, hasRadius.exactly(1, float)]

CylinderShape.is_a = [ShapeRegion, hasRadius.some(float), hasLength.exactly(1, float)]

Classifier.is_a = [StatisticalReasoner]

StatisticalReasoner.is_a = [Reasoner]

ClausalObject.is_a = [Phrase]

Phrase.is_a = [LinguisticObject]

Clean.is_a = [CleanlinessRegion]

CleanlinessRegion.is_a = [Region]

Cleaning.is_a = [ModifyingPhysicalObject, isTaskAffordedBy.some(Purification)]

ModifyingPhysicalObject.is_a = [PhysicalTask]

Purification.is_a = [Variability, affordsSetpoint.only(classifies.only(Clean & isRegionFor.only(Cleanliness))), affordsTrigger.only(classifies.only(PhysicalObject))]

Cleanliness.is_a = [SocialQuality, hasRegion.some(CleanlinessRegion), hasRegion.only(CleanlinessRegion)]

SocialQuality.is_a = [Quality]

ClientServer_Specification.is_a = [InterfaceSpecification, definesRole.some(ClientRole) & definesRole.some(ServerRole)]

ClientRole.is_a = [InterfaceComponentRole, isDefinedIn.some(ClientServer_Specification)]

ServerRole.is_a = [InterfaceComponentRole, isDefinedIn.some(ClientServer_Specification)]

InterfaceComponentRole.is_a = [SoftwareRole]
InterfaceComponentRole.equivalent_to = [SoftwareRole & isDefinedIn.only(InterfaceSpecification)]

Closing.is_a = [Actuating]

Delivering.is_a = [Actuating, isTaskAffordedBy.some(Shifting)]

Fetching.is_a = [PhysicalAcquiring]

Lifting.is_a = [Actuating]

Opening.is_a = [Actuating]

Pulling.is_a = [Actuating]

Pushing.is_a = [Actuating]

Squeezing.is_a = [Actuating]

Clumsiness.is_a = [Amateurish]

CognitiveAgent.is_a = [Agent]

SubCognitiveAgent.is_a = [Agent]

Collision.is_a = [ProcessType]

Extrinsic.is_a = [PhysicalQuality]

ImperativeClause.is_a = [ClausalObject, expresses.some(StateTransition), expresses.only(StateTransition)]

CommitedObject.is_a = [ConnectedObject]

ConnectedObject.is_a = [Patient]

CommunicationAction.is_a = [Action, hasParticipant.some(LinguisticObject)]

LinguisticObject.is_a = [InformationObject]

CommunicationReport.is_a = [CommunicationTask]

Receiver.is_a = [ExperiencerRole, hasTask.some(CommunicationTask)]

Sender.is_a = [PerformerRole, hasTask.some(CommunicationTask)]

SocialObject.is_a = [Object, isExpressedBy.some(InformationObject), hasPart.only(SocialObject)]

CommunicationTopic.is_a = [ResourceRole, classifies.only(SocialObject & isParticipantIn.some(isClassifiedBy.some(CommunicationTask)))]

ResourceRole.is_a = [EventAdjacentRole]

Composing.is_a = [Variability, affordsTrigger.only(ConnectedObject)]

Computer_Language.is_a = [FormalLanguage]

FormalLanguage.is_a = [Language, givesMeaningTo.only(Structured_Text)]

Computer_Program.is_a = [Thing]
Computer_Program.equivalent_to = [Structured_Text & isGivenMeaningBy.some(Programming_Language), Structured_Text & expresses.some(Algorithm)]

Programming_Language.is_a = [Computer_Language, givesMeaningTo.only(Computer_Program)]

Conclusion.is_a = [CreatedObject, Knowledge]

CreatedObject.is_a = [Patient]

Knowledge.is_a = [Item]

ConditionalSuccedence.is_a = [Succedence, hasBinding.only(CounterfactualBinding)]

Configuration.is_a = [Description, describes.some(StateType)]

Connectivity.is_a = [Disposition, affordsBearer.only(ConnectedObject), affordsTrigger.only(ConnectedObject)]

ContactState.is_a = [StateType, classifies.only(hasParticipant.min(2, PhysicalObject))]

Container.is_a = [Restrictor]

Containment.is_a = [Disposition, affordsBearer.only(Container), affordsTrigger.only(IncludedObject), isDispositionOf.only(hasQuality.some(Capacity))]

IncludedObject.is_a = [Patient]

ContainmentState.is_a = [FunctionalControl, isStateTypeOf.some(Container & classifies.only(hasDisposition.some(Containment)))]

FunctionalControl.is_a = [StateType, isStateTypeOf.some(Item), isStateTypeOf.some(Restrictor)]

ContainmentTheory.is_a = [ControlTheory]

ControlTheory.is_a = [FunctionalSpatialSchemaTheory]

ContinuousJoint.is_a = [HingeJoint]

HingeJoint.is_a = [MovableJoint]

FunctionalSpatialSchemaTheory.is_a = [ImageSchemaTheory, defines.some(LocatumRole), defines.some(RelatumRole)]

Cover.is_a = [Barrier]

Coverage.is_a = [Blockage, affordsBearer.only(Cover), affordsTrigger.only(CoveredObject)]

CoveredObject.is_a = [BlockedObject]

CoverageTheory.is_a = [FunctionalSpatialSchemaTheory]

CoveringTheory.is_a = [ExecutableSchematicTheory, defines.some(Instrument), defines.exactly(1, Patient)]

ExecutableSchematicTheory.is_a = [SchematicTheory, defines.some(PerformerRole)]

CrackingTheory.is_a = [ExecutableSchematicTheory, defines.some(Instrument), defines.some(Patient)]

Creation.is_a = [ProcessType, isProcessTypeOf.some(CreatedObject)]

Cuttability.is_a = [Disposition, affordsBearer.only(CutObject), affordsTrigger.only(Cutter)]

Tool.is_a = [Instrument]

Cutting.is_a = [ModifyingPhysicalObject]

Database.is_a = [SoftwareRole]

SoftwareRole.is_a = [Role, classifies.only(Software)]

Deciding.is_a = [DerivingInformation]

DerivingInformation.is_a = [InformationAcquisition, isTaskOfInputRole.some(Premise) & isTaskOfOutputRole.some(Conclusion)]

DeductiveReasoning.is_a = [Reasoning]

Deformation.is_a = [Alteration, isProcessTypeOf.exactly(1, ShapedObject)]

ShapedObject.is_a = [AlteredObject, isTriggerDefinedIn.some(describesQuality.only(Shape)), classifies.only(hasQuality.some(Shape))]

FluidFlow.is_a = [Motion, isProcessTypeOf.some(MovedObject)]

Shifting.is_a = [Variability, affordsSetpoint.only(classifies.only(Localization)), affordsTrigger.only(MovedObject)]

DependentPlace.is_a = [Feature]

Deposit.is_a = [Instrument]

DepositedObject.is_a = [Patient]

Deposition.is_a = [Disposition, affordsBearer.only(Deposit), affordsTrigger.only(DepositedObject)]

InformationAcquisition.is_a = [Thing]
InformationAcquisition.equivalent_to = [MentalTask & isTaskOfOutputRole.some(Knowledge)]

Premise.is_a = [Knowledge]

DesignedComponent.is_a = [FunctionalPart, DesignedArtifact, hasDisposition.some(Connectivity)]

FunctionalPart.is_a = [PhysicalBody, isComponentOf.only(Agent | DesignedArtifact)]

DesignedContainer.is_a = [DesignedArtifact, hasDisposition.some(Containment)]

DesignedFurniture.is_a = [DesignedArtifact]

DesignedTool.is_a = [DesignedArtifact]

Destination.is_a = [Location]

Location.is_a = [SpatioTemporalRole]

DestroyedObject.is_a = [Patient]

Destruction.is_a = [ProcessType, isProcessTypeOf.some(DestroyedObject)]

DetectedObject.is_a = [Patient]

DeviceState.is_a = [Intrinsic]

DeviceStateRange.is_a = [Region]
DeviceStateRange.equivalent_to = [DeviceTurnedOff | DeviceTurnedOn]

DeviceTurnedOff.is_a = [DeviceStateRange]

DeviceTurnedOn.is_a = [DeviceStateRange]

Dicing.is_a = [Cutting]

DirectedMotion.is_a = [Motion]

UndirectedMotion.is_a = [Motion]

Dirty.is_a = [CleanlinessRegion]

Discourse.is_a = [CommunicationTask]

Distancing.is_a = [Navigating]

Dreaming.is_a = [DerivingInformation]

Driving.is_a = [Locomotion]

Flying.is_a = [Locomotion]

Swimming.is_a = [Locomotion]

Walking.is_a = [Locomotion]

Dropping.is_a = [Actuating]

Placing.is_a = [PhysicalTask]

ESTSchemaTheory.is_a = [ImageSchemaTheory, defines.some(ExistingObjectRole), defines.exactly(1, Thing)]

ExistingObjectRole.is_a = [RelationAdjacentRole, classifies.only(PhysicalObject)]

Effort.is_a = [Parameter]

EnclosedObject.is_a = [IncludedObject]

Enclosing.is_a = [Containment, affordsTrigger.only(EnclosedObject)]

EndEffectorPositioning.is_a = [Manipulating]

Manipulating.is_a = [PhysicalTask, classifies.only(PhysicalAction)]

Episode.is_a = [Situation]

ExcludedObject.is_a = [Patient]

ExecutableFile.is_a = [Thing]
ExecutableFile.equivalent_to = [Digital_File & realizes.some(Executable_Code)]

Executable_Code.is_a = [Computer_Program]
Executable_Code.equivalent_to = [Computer_Program & isGivenMeaningBy.some(ExecutableFormat)]

ExecutableFormat.is_a = [File_format, givesMeaningTo.only(Executable_Code)]

SchematicTheory.is_a = [Theory]

ExecutableSoftware.is_a = [Thing]
ExecutableSoftware.equivalent_to = [Software & hasMember.some(ExecutableFile)]

Software.is_a = [Design, describes.some(Software_Configuration), describes.only(SoftwareInstance | Software_Configuration)]

RelationAdjacentRole.is_a = [Role]

ExperiencerRole.is_a = [PerformerRole]

ExtractedObject.is_a = [Patient]

PhysicalQuality.is_a = [Quality, isQualityOf.exactly(1, PhysicalObject)]

FailedAttempt.is_a = [Unsuccessfulness]

Unsuccessfulness.is_a = [SuccessDiagnosis]

FaultySoftware.is_a = [SoftwareDiagnosis]

SoftwareDiagnosis.is_a = [TechnicalDiagnosis]

PhysicalAcquiring.is_a = [ModifyingPhysicalObject]

Configuration.is_a = [Collection]

Finger.is_a = [Limb, isPartOf.some(Hand)]

Hand.is_a = [PrehensileEffector]

FixedJoint.is_a = [Joint, hasJointState.exactly(0, Entity)]

MovableJoint.is_a = [Joint, hasJointState.exactly(1, JointState)]

Flipping.is_a = [Actuating, isTaskAffordedBy.some(Shifting)]

FloatingJoint.is_a = [MovableJoint]

Fluid.is_a = [Substance]

Substance.is_a = [PhysicalBody]

MovedObject.is_a = [AlteredObject, isTriggerDefinedIn.some(describesQuality.only(Localization)), classifies.only(hasQuality.some(Localization))]

Focusing.is_a = [AttentionShift]

Foolishness.is_a = [Amateurish]

ForgettingIncorrectInformation.is_a = [InformationDismissal]

InformationDismissal.is_a = [MentalTask, isTaskOfInputRole.some(ExcludedObject & Knowledge)]

ForgettingIrrelevantInformation.is_a = [InformationDismissal]

Language.is_a = [System, givesMeaningTo.only(Text)]

Item.is_a = [Patient]

FunctionalDesign.is_a = [Design]

FunctionalDiagnosis.is_a = [Diagnosis]

PhysicalBody.is_a = [PhysicalObject]

LocatumRole.is_a = [SpatialRelationRole, classifies.only(PhysicalObject)]

RelatumRole.is_a = [SpatialRelationRole, classifies.only(PhysicalObject)]

GetTaskParameter.is_a = [Planning]

Planning.is_a = [Deciding, isTaskOf.some(classifies.some(Plan))]

GraphDatabase.is_a = [Database]

GraphQueryLanguage.is_a = [QueryLanguage]

QueryLanguage.is_a = [Computer_Language]

GraspTransfer.is_a = [Grasping]

Grasping.is_a = [Manipulating]

Graspability.is_a = [Disposition]

Releasing.is_a = [Manipulating]

GraspingMotion.is_a = [PrehensileMotion]
GraspingMotion.equivalent_to = [IntermediateGrasp | PowerGrasp | PrecisionGrasp]

IntermediateGrasp.is_a = [GraspingMotion]

PowerGrasp.is_a = [GraspingMotion]

PrecisionGrasp.is_a = [GraspingMotion]

PrehensileMotion.is_a = [DirectedMotion, isProcessTypeOf.some(MovedObject & classifies.only(PrehensileEffector))]
PrehensileMotion.equivalent_to = [GraspingMotion | ReleasingMotion]

ReleasingMotion.is_a = [PrehensileMotion]

GreenColor.is_a = [ColorRegion]

Gripper.is_a = [PrehensileEffector]

PrehensileEffector.is_a = [PhysicalEffector]

HardwareDiagnosis.is_a = [TechnicalDiagnosis]

TechnicalDiagnosis.is_a = [FunctionalDiagnosis, hasConstituent.some(DesignedArtifact)]

HasQualityRegion.is_a = [Relation, hasQuality.exactly(1, Quality), hasRegion.exactly(1, Region)]

Head.is_a = [FunctionalPart]

HeadMovement.is_a = [BodyMovement]

HeadTurning.is_a = [HeadMovement]

Holding.is_a = [Manipulating]

HostRole.is_a = [InterfaceComponentRole, isDefinedIn.some(PluginSpecification)]

PluginSpecification.is_a = [API_Specification, definesRole.some(HostRole) & definesRole.some(PluginRole)]

Humanreadable_Programming_Language.is_a = [Programming_Language, givesMeaningTo.only(Source_Code)]

Source_Code.is_a = [Computer_Program]
Source_Code.equivalent_to = [Computer_Program & isGivenMeaningBy.some(Humanreadable_Programming_Language)]

HumanActivityRecording.is_a = [RecordedEpisode]

RecordedEpisode.is_a = [Episode, includesRecord.some(InformationObject)]

Imagining.is_a = [InformationAcquisition]

Impediment.is_a = [Blockage, affordsBearer.only(Obstacle), affordsTrigger.only(RestrictedObject)]

Obstacle.is_a = [Barrier]

RestrictedObject.is_a = [BlockedObject]

Inability.is_a = [Unsuccessfulness]

IncompatibleSoftware.is_a = [SoftwareDiagnosis]

InductiveReasoning.is_a = [Reasoning]

Infeasibility.is_a = [Unsuccessfulness]

InferenceRules.is_a = [Knowledge]

InformationRetrieval.is_a = [InformationAcquisition, isTaskOfOutputRole.some(Knowledge)]

InformationStorage.is_a = [MentalTask, isTaskOfInputRole.some(Knowledge & StoredObject)]

StoredObject.is_a = [EnclosedObject]

InsertedObject.is_a = [EnclosedObject]

Insertion.is_a = [Enclosing, affordsTrigger.only(InsertedObject)]

Instructions.is_a = [Item]

Interpreting.is_a = [DerivingInformation]

InterrogativeClause.is_a = [ClausalObject]

Introspecting.is_a = [InformationAcquisition]

KineticFrictionAttribute.is_a = [FrictionAttribute]

KinoDynamicData.is_a = [InformationObject, isAbout.only(PhysicalObject)]

KnowledgeRepresentationLanguage.is_a = [Computer_Language]

Labeling.is_a = [Interpreting]

Text.is_a = [Thing]
Text.equivalent_to = [InformationObject & isGivenMeaningBy.some(Language)]

Leaning.is_a = [PosturalMoving]

PosturalMoving.is_a = [BodyMovement]

Learning.is_a = [InformationStorage]

Leg.is_a = [Limb]

LimbMotion.is_a = [DirectedMotion, isProcessTypeOf.some(MovedObject & classifies.only(Limb))]

Linkage.is_a = [Connectivity, affordsBearer.only(LinkedObject), affordsTrigger.only(LinkedObject)]

LinkedObject.is_a = [ConnectedObject]

LinkageState.is_a = [StateType, isStateTypeOf.some(LinkedObject)]

SpatioTemporalRole.is_a = [EventAdjacentRole]

SpatialRelationRole.is_a = [RelationAdjacentRole]

LocutionaryAction.is_a = [Thing]
LocutionaryAction.equivalent_to = [CommunicationAction & hasParticipant.some(Agent)]

LookingAt.is_a = [PhysicalTask]

LookingFor.is_a = [Perceiving]

Lowering.is_a = [Actuating]

PhysicalAction.is_a = [Action]

Markup_Language.is_a = [Computer_Language]

Material.is_a = [Intrinsic]

MedicalDiagnosis.is_a = [FunctionalDiagnosis, hasConstituent.some(Organism)]

Organism.is_a = [BiologicalObject, PhysicalAgent]

Memorizing.is_a = [Learning]

MentalAction.is_a = [Action, hasParticipant.only(Agent | SocialObject)]

MeshShape.is_a = [ShapeRegion, hasFilePath.exactly(1, str), hasShapeScale.max(1, float)]

MeshShapeData.is_a = [InformationObject, isAbout.only(PhysicalObject)]

MetaCognitionEvaluationTopic.is_a = [MetaCognitionTopic]

MetaCognitionTopic.is_a = [ThinkAloudTopic]

MetaCognitionMemoryTopic.is_a = [MetaCognitionTopic]

MetaCognitionPlanningTopic.is_a = [MetaCognitionTopic]

ThinkAloudTopic.is_a = [CommunicationTopic]

MetacognitiveControlling.is_a = [MentalTask]

MetacognitiveMonitoring.is_a = [Introspecting]

Mixing.is_a = [Constructing]

MixingTheory.is_a = [ExecutableSchematicTheory, defines.some(Instrument), defines.some(Patient)]

MonitoringJointState.is_a = [Proprioceiving]

Proprioceiving.is_a = [PhysicalTask]

ProcessFlow.is_a = [Description, definesProcess.some(ProcessType)]

PhysicsProcess.is_a = [Process, hasParticipant.some(PhysicalObject)]

MovingAway.is_a = [Locomotion]

MovingTo.is_a = [Navigating]

Natural_Language.is_a = [Language, givesMeaningTo.only(Natural_Language_Text)]

Natural_Language_Text.is_a = [Thing]
Natural_Language_Text.equivalent_to = [Text & isGivenMeaningBy.some(Natural_Language)]

Ontology.is_a = [Structured_Text]
Ontology.equivalent_to = [Structured_Text & isGivenMeaningBy.some(Ontology_Language)]

Ontology_Language.is_a = [KnowledgeRepresentationLanguage, givesMeaningTo.only(Ontology)]

Option.is_a = [ResourceRole]

FormalEntity.is_a = [Abstract]

Singleton.is_a = [Thing]
Singleton.equivalent_to = [Set & encapsulates.exactly(1, Entity)]

Orienting.is_a = [Positioning]

Positioning.is_a = [Actuating]

Origin.is_a = [Location]

ParkingArms.is_a = [AssumingArmPose]

PhaseTransition.is_a = [Alteration]

PhysicalAccessibility.is_a = [StateType, isStateTypeOf.some(Item), isStateTypeOf.some(Restrictor)]

PhysicalBlockage.is_a = [StateType, isStateTypeOf.some(Item), isStateTypeOf.some(Restrictor)]

PhysicalExistence.is_a = [PhysicalState]

PhysicalState.is_a = [State, hasParticipant.some(PhysicalObject)]

PlacingTheory.is_a = [ExecutableSchematicTheory, defines.some(Destination), defines.some(Patient)]

PlanarJoint.is_a = [MovableJoint]

PluginRole.is_a = [InterfaceComponentRole, isDefinedIn.some(PluginSpecification)]

Pourable.is_a = [Disposition, affordsBearer.only(PouredObject)]

PouredObject.is_a = [Patient]

Pouring.is_a = [Actuating, isTaskAffordedBy.some(Pourable)]

PouringInto.is_a = [Pouring]

PouringOnto.is_a = [Pouring]

Prediction.is_a = [Prospecting]

Prospecting.is_a = [DerivingInformation]

Predilection.is_a = [SocialRelation, describes.only(Preference | (Order & orders.only(encapsulates.only(Situation))))]

SocialRelation.is_a = [Relation]

PreferenceOrder.is_a = [SocialRelation, orders.some(OrderedElement), orders.only(encapsulates.only(Description)), describes.only(Preference)]

PreferenceRegion.is_a = [SocialObjectAttribute, isRegionFor.some(Preference)]

SocialObjectAttribute.is_a = [Region, isRegionFor.only(SocialObject)]

PrismaticJoint.is_a = [MovableJoint, hasJointLimit.exactly(1, JointLimit)]

Progression.is_a = [Situation]
Progression.equivalent_to = [satisfies.some(ProcessFlow)]

Protector.is_a = [Restrictor]

ProximalTheory.is_a = [ImageSchemaTheory, defines.some(LocatumRole), defines.some(RelatumRole)]

PushingAway.is_a = [Pushing]

PushingDown.is_a = [Pushing]

PuttingDown.is_a = [Manipulating]

QualityTransition.is_a = [Transition]

Query.is_a = [Message]

QueryAnsweringTask.is_a = [Thing]
QueryAnsweringTask.equivalent_to = [AnsweringTask & classifies.only(isReactionTo.only(isClassifiedBy.some(QueryingTask)))]

QueryEngine.is_a = [SoftwareRole]

Reaching.is_a = [EndEffectorPositioning]

Retracting.is_a = [EndEffectorPositioning]

Reasoner.is_a = [SoftwareRole]

RecipientRole.is_a = [BeneficiaryRole]

RedColor.is_a = [ColorRegion]

Reification.is_a = [Description, isReificationOf.exactly(1, normstr)]

RelationalDatabase.is_a = [Database]

RelationalQueryLanguage.is_a = [QueryLanguage]

RelevantPart.is_a = [Feature]

Remembering.is_a = [Thing, InformationRetrieval & Retrospecting]

Retrospecting.is_a = [InformationAcquisition]

RemovedObject.is_a = [ExcludedObject]

Replanning.is_a = [Planning]

RevoluteJoint.is_a = [HingeJoint, hasJointLimit.exactly(1, JointLimit)]

Room.is_a = [PhysicalPlace]

RoomSurface.is_a = [Surface]

Surface.is_a = [PhysicalPlace]

Rubbing.is_a = [DirectedMotion]

Theory.is_a = [Description, hasComponent.some(Relation)]

SelectedObject.is_a = [Role]

Selecting.is_a = [Deciding]

SelectingItem.is_a = [GetTaskParameter, isTaskOfOutputRole.some(SelectedObject & classifies.only(PhysicalObject))]

SelfReflection.is_a = [MetacognitiveControlling]

Serving.is_a = [Delivering, classifies.only(Action & (hasParticipant.some(PhysicalAgent) | hasParticipant.some(PhysicalObject)))]

SettingGripper.is_a = [AssumingPose]

SixDPose.is_a = [SpaceRegion, hasPositionData.exactly(1, str)]

Shaping.is_a = [Variability, affordsSetpoint.only(classifies.only(Shape)), affordsTrigger.only(ShapedObject)]

Sharpness.is_a = [Intrinsic]

Simulating.is_a = [Prospecting]

Simulation_Reasoner.is_a = [Reasoner]

Set.is_a = [FormalEntity]

Size.is_a = [Intrinsic]

Slicing.is_a = [Cutting]

Sluggishness.is_a = [Amateurish]

SocialState.is_a = [State, hasParticipant.some(Agent)]

Software_Configuration.is_a = [Configuration, isDescribedBy.exactly(1, Software)]

SocialAgent.is_a = [Agent, SocialObject, actsThrough.some(PhysicalAgent)]

SoftwareLibrary.is_a = [Software]

SourceMaterialRole.is_a = [ResourceRole]

SphereShape.is_a = [ShapeRegion, hasRadius.exactly(1, float)]

Standing.is_a = [PosturalMoving]

StaticFrictionAttribute.is_a = [FrictionAttribute]

StatusFailure.is_a = [Region]

StimulusRole.is_a = [CausativeRole]

Stirring.is_a = [Mixing, isTaskAffordedBy.some(Composing)]

Storage.is_a = [Enclosing, affordsTrigger.only(StoredObject)]

StructuralDesign.is_a = [Design]

TaskInvocation.is_a = [Workflow, hasBinding.only(FactualBinding), definesTask.exactly(1, Task)]

SuccessDiagnosis.is_a = [BehavioralDiagnosis]

Successfulness.is_a = [SuccessDiagnosis]

SupportState.is_a = [FunctionalControl, isStateTypeOf.some(Supporter & classifies.only(hasDisposition.some(Deposition)))]

Supporter.is_a = [Restrictor]

SupportTheory.is_a = [ControlTheory]

SupportedObject.is_a = [ConnectedObject]

SymbolicReasoner.is_a = [Reasoner]

Tapping.is_a = [DirectedMotion]

Taxis.is_a = [BodyMovement]

Temperature.is_a = [Intrinsic, hasRegion.some(TemperatureRegion), hasRegion.only(TemperatureRegion)]

TemperatureRegion.is_a = [Region]

Tempering.is_a = [Variability, affordsSetpoint.only(classifies.only(Temperature))]

ThinkAloud.is_a = [CommunicationReport]

ThinkAloudActionTopic.is_a = [ThinkAloudTopic]

ThinkAloudGeneralKnowledgeTopic.is_a = [ThinkAloudKnowledgeTopic]

ThinkAloudKnowledgeTopic.is_a = [ThinkAloudTopic]

ThinkAloudObstructionTopic.is_a = [ThinkAloudTopic]

ThinkAloudOpinionTopic.is_a = [ThinkAloudTopic]

ThinkAloudPerceptionTopic.is_a = [ThinkAloudTopic]

ThinkAloudPlanTopic.is_a = [ThinkAloudTopic]

ThinkAloudSceneKnowledgeTopic.is_a = [ThinkAloudKnowledgeTopic]

Threshold.is_a = [Parameter]

Throwing.is_a = [Actuating, Manipulating]

TimeRole.is_a = [SpatioTemporalRole]

Transporting.is_a = [ModifyingPhysicalObject]

Triplestore.is_a = [GraphDatabase]

Turning.is_a = [PosturalMoving]

UnavailableSoftware.is_a = [SoftwareDiagnosis]

VideoData.is_a = [InformationObject]

ThreeDPosition.is_a = [SpaceRegion, hasPositionData.exactly(1, str)]

hasColorValue.is_a = [DatatypeProperty, hasRegionDataValue]
hasColorValue.domain = [ColorRegion]

hasRegionDataValue.is_a = [DatatypeProperty, hasDataValue]
hasRegionDataValue.domain = [Region]

hasDataFormat.is_a = [DatatypeProperty, hasDataValue]
hasDataFormat.domain = [InformationRealization]
hasDataFormat.range = [str]

hasDataValue.is_a = [DatatypeProperty]
hasDataValue.domain = [Entity]

hasDepth.is_a = [DatatypeProperty, hasShapeParameter]
hasDepth.domain = [ShapeRegion]
hasDepth.range = [float]

hasShapeParameter.is_a = [DatatypeProperty, hasRegionDataValue]
hasShapeParameter.domain = [ShapeRegion]
hasShapeParameter.range = [float | float | float | str]

hasEventBegin.is_a = [DatatypeProperty, hasEventTime]
hasEventBegin.domain = [Event]
hasEventBegin.range = [float]

hasEventTime.is_a = [DatatypeProperty, hasDataValue]
hasEventTime.domain = [Event]
hasEventTime.range = [float]

hasEventEnd.is_a = [DatatypeProperty, hasEventTime]
hasEventEnd.domain = [Event]
hasEventEnd.range = [float]

hasFilePath.is_a = [DatatypeProperty, hasDataValue]
hasFilePath.domain = [Entity]
hasFilePath.range = [str]

hasForceValue.is_a = [DatatypeProperty, hasRegionDataValue]
hasForceValue.domain = [ForceAttribute]
hasForceValue.range = [float]

hasFrictionValue.is_a = [DatatypeProperty, hasRegionDataValue]
hasFrictionValue.domain = [FrictionAttribute]
hasFrictionValue.range = [float]

hasHSVValue.is_a = [DatatypeProperty, hasColorValue]
hasHSVValue.domain = [ColorRegion]
hasHSVValue.range = [str]

hasHeight.is_a = [DatatypeProperty, hasShapeParameter]
hasHeight.domain = [ShapeRegion]
hasHeight.range = [float]

hasIntervalBegin.is_a = [DatatypeProperty, hasIntervalTime]
hasIntervalBegin.domain = [TimeInterval]
hasIntervalBegin.range = [float]

hasIntervalTime.is_a = [DatatypeProperty, hasRegionDataValue]
hasIntervalTime.domain = [TimeInterval]
hasIntervalTime.range = [float]

hasIntervalEnd.is_a = [DatatypeProperty, hasIntervalTime]
hasIntervalEnd.domain = [TimeInterval]
hasIntervalEnd.range = [float]

hasJointEffort.is_a = [DatatypeProperty, hasJointParameter]
hasJointEffort.domain = [JointState]
hasJointEffort.range = [float]

hasJointParameter.is_a = [DatatypeProperty, hasRegionDataValue]
hasJointParameter.domain = [PhysicalAttribute]
hasJointParameter.range = [float]

hasJointEffortLimit.is_a = [DatatypeProperty, hasJointParameter]
hasJointEffortLimit.domain = [JointLimit]
hasJointEffortLimit.range = [float]

hasJointPosition.is_a = [DatatypeProperty, hasJointParameter]
hasJointPosition.domain = [JointState]
hasJointPosition.range = [float]

hasJointPositionMax.is_a = [DatatypeProperty, hasJointParameter]
hasJointPositionMax.domain = [JointLimit]
hasJointPositionMax.range = [float]

hasJointPositionMin.is_a = [DatatypeProperty, hasJointParameter]
hasJointPositionMin.domain = [JointLimit]
hasJointPositionMin.range = [float]

hasJointVelocity.is_a = [DatatypeProperty, hasJointParameter]
hasJointVelocity.domain = [JointState]
hasJointVelocity.range = [float]

hasJointVelocityLimit.is_a = [DatatypeProperty, hasJointParameter]
hasJointVelocityLimit.domain = [JointLimit]
hasJointVelocityLimit.range = [float]

hasLength.is_a = [DatatypeProperty, hasShapeParameter]
hasLength.domain = [ShapeRegion]
hasLength.range = [float]

hasMassValue.is_a = [DatatypeProperty, hasRegionDataValue]
hasMassValue.domain = [MassAttribute]
hasMassValue.range = [float]

hasNameString.is_a = [DatatypeProperty, hasDataValue]
hasNameString.domain = [Entity]
hasNameString.range = [str]

hasPersistentIdentifier.is_a = [DatatypeProperty, hasDataValue]
hasPersistentIdentifier.domain = [InformationRealization]
hasPersistentIdentifier.range = [str]

hasPositionData.is_a = [DatatypeProperty, hasSpaceParameter]
hasPositionData.domain = [SpaceRegion]
hasPositionData.range = [str]

hasSpaceParameter.is_a = [DatatypeProperty, hasRegionDataValue]
hasSpaceParameter.domain = [SpaceRegion]

hasPriority.is_a = [DatatypeProperty, hasDataValue]
hasPriority.domain = [Task]

hasRGBValue.is_a = [DatatypeProperty, hasColorValue]
hasRGBValue.domain = [ColorRegion]
hasRGBValue.range = [str]

hasRadius.is_a = [DatatypeProperty, hasShapeParameter]
hasRadius.domain = [ShapeRegion]
hasRadius.range = [float]

hasReferenceFrame.is_a = [DatatypeProperty, hasSpaceParameter]
hasReferenceFrame.domain = [SpaceRegion]
hasReferenceFrame.range = [str]

hasShapeScale.is_a = [DatatypeProperty, hasShapeParameter]
hasShapeScale.domain = [ShapeRegion]
hasShapeScale.range = [float]

hasWidth.is_a = [DatatypeProperty, hasShapeParameter]
hasWidth.domain = [ShapeRegion]
hasWidth.range = [float]

isReificationOf.is_a = [DatatypeProperty, hasDataValue]
isReificationOf.domain = [Description]
isReificationOf.range = [normstr]

affects.is_a = [ObjectProperty, precedes]

precedes.is_a = [ObjectProperty, TransitiveProperty, associatedWith]
precedes.domain = [Entity]
precedes.range = [Entity]

isAffectedBy.is_a = [ObjectProperty, follows]

affordanceDefines.is_a = [ObjectProperty, defines]
affordanceDefines.domain = [Affordance]
affordanceDefines.range = [Concept]

defines.is_a = [ObjectProperty, usesConcept]
defines.domain = [Description]
defines.range = [Concept]

isDefinedInAffordance.is_a = [ObjectProperty, isDefinedIn]
isDefinedInAffordance.domain = [Concept]
isDefinedInAffordance.range = [Affordance]

affordanceDefinesTask.is_a = [ObjectProperty, affordanceDefines, definesTask]
affordanceDefinesTask.domain = [Affordance]
affordanceDefinesTask.range = [Task]

definesTask.is_a = [ObjectProperty, defines]
definesTask.domain = [Description]
definesTask.range = [Task]

isTaskDefinedInAffordance.is_a = [ObjectProperty, isDefinedInAffordance, isTaskDefinedIn]
isTaskDefinedInAffordance.domain = [Task]
isTaskDefinedInAffordance.range = [Affordance]

affordsBearer.is_a = [ObjectProperty, affordsConcept]
affordsBearer.domain = [Disposition]
affordsBearer.range = [Role]

affordsConcept.is_a = [ObjectProperty, associatedWith]
affordsConcept.domain = [Disposition]
affordsConcept.range = [Concept]

isBearerAffordedBy.is_a = [ObjectProperty, isConceptAffordedBy]
isBearerAffordedBy.domain = [Role]
isBearerAffordedBy.range = [Disposition]

isDescribedBy.is_a = [ObjectProperty, associatedWith]
isDescribedBy.domain = [Entity]
isDescribedBy.range = [Description]

definesBearer.is_a = [ObjectProperty, definesRole]
definesBearer.domain = [Affordance]
definesBearer.range = [Role]

associatedWith.is_a = [ObjectProperty, SymmetricProperty, TransitiveProperty]
associatedWith.domain = [Entity]
associatedWith.range = [Entity]

isConceptAffordedBy.is_a = [ObjectProperty, associatedWith]
isConceptAffordedBy.domain = [Concept]
isConceptAffordedBy.range = [Disposition]

affordsPerformer.is_a = [ObjectProperty, affordsConcept]
affordsPerformer.domain = [Disposition]
affordsPerformer.range = [Role]

isPerformerAffordedBy.is_a = [ObjectProperty, isConceptAffordedBy]
isPerformerAffordedBy.domain = [Role]
isPerformerAffordedBy.range = [Disposition]

definesPerformer.is_a = [ObjectProperty, definesRole]
definesPerformer.domain = [Affordance]
definesPerformer.range = [Role]

affordsSetpoint.is_a = [ObjectProperty, affordsConcept]
affordsSetpoint.domain = [Disposition]
affordsSetpoint.range = [Setpoint]

isSetpointAffordedBy.is_a = [ObjectProperty, isConceptAffordedBy]
isSetpointAffordedBy.domain = [Setpoint]
isSetpointAffordedBy.range = [Disposition]

definesSetpoint.is_a = [ObjectProperty, definesParameter]
definesSetpoint.domain = [Description]
definesSetpoint.range = [Setpoint]

affordsTask.is_a = [ObjectProperty, affordsConcept]
affordsTask.domain = [Disposition]
affordsTask.range = [Task]

isTaskAffordedBy.is_a = [ObjectProperty, isConceptAffordedBy]
isTaskAffordedBy.domain = [Task]
isTaskAffordedBy.range = [Disposition]

affordsTrigger.is_a = [ObjectProperty, affordsConcept]
affordsTrigger.domain = [Disposition]
affordsTrigger.range = [Role]

isTriggerAffordedBy.is_a = [ObjectProperty, isConceptAffordedBy]
isTriggerAffordedBy.domain = [Role]
isTriggerAffordedBy.range = [Disposition]

definesTrigger.is_a = [ObjectProperty, definesRole]
definesTrigger.domain = [Affordance]
definesTrigger.range = [Role]

after.is_a = [ObjectProperty, TransitiveProperty, follows]
after.domain = [Entity]
after.range = [Entity]

follows.is_a = [ObjectProperty, TransitiveProperty, associatedWith]
follows.domain = [Entity]
follows.range = [Entity]

before.is_a = [ObjectProperty, TransitiveProperty, precedes]
before.domain = [Entity]
before.range = [Entity]

answers.is_a = [ObjectProperty, relatesToAnotherRole]
answers.domain = [Answer]
answers.range = [Message]

relatesToAnotherRole.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
relatesToAnotherRole.domain = [Role]
relatesToAnotherRole.range = [Role]

hasAnswer.is_a = [ObjectProperty, relatesToAnotherRole]
hasAnswer.domain = [Message]
hasAnswer.range = [Answer]

causes.is_a = [ObjectProperty, TransitiveProperty, affects]

isReactionTo.is_a = [ObjectProperty, TransitiveProperty, isAffectedBy]
isReactionTo.domain = [Event]
isReactionTo.range = [Event]

causesTransition.is_a = [ObjectProperty, isEventIncludedIn]
causesTransition.domain = [Event]
causesTransition.range = [Transition]

isEventIncludedIn.is_a = [ObjectProperty, hasSetting]
isEventIncludedIn.domain = [Event]
isEventIncludedIn.range = [Situation]

isCausedByEvent.is_a = [ObjectProperty, includesEvent]
isCausedByEvent.domain = [Transition]
isCausedByEvent.range = [Event]

coOccurs.is_a = [ObjectProperty, SymmetricProperty, overlaps]
coOccurs.domain = [Event]
coOccurs.range = [Event]

overlaps.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
overlaps.domain = [Entity]
overlaps.range = [Entity]

contains.is_a = [ObjectProperty, overlaps]
contains.domain = [Entity]
contains.range = [Entity]

isContainedIn.is_a = [ObjectProperty, overlaps]
isContainedIn.domain = [Entity]
isContainedIn.range = [Entity]

containsEvent.is_a = [ObjectProperty, TransitiveProperty, coOccurs, contains]
containsEvent.domain = [Event]
containsEvent.range = [Event]

during.is_a = [ObjectProperty, TransitiveProperty, coOccurs, isContainedIn]
during.domain = [Event]
during.range = [Event]

containsObject.is_a = [ObjectProperty, TransitiveProperty, contains, isLocationOf]
containsObject.domain = [PhysicalObject]
containsObject.range = [PhysicalObject]

isLocationOf.is_a = [ObjectProperty, associatedWith]
isLocationOf.domain = [Entity]
isLocationOf.range = [Entity]

isInsideOf.is_a = [ObjectProperty, TransitiveProperty, isContainedIn, hasLocation]
isInsideOf.domain = [PhysicalObject]
isInsideOf.range = [PhysicalObject]

coversObject.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, interactsWith]
coversObject.domain = [PhysicalObject]
coversObject.range = [PhysicalObject]

interactsWith.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
interactsWith.domain = [PhysicalObject]
interactsWith.range = [PhysicalObject]

isCoveredByObject.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, interactsWith]
isCoveredByObject.domain = [PhysicalObject]
isCoveredByObject.range = [PhysicalObject]

definesRole.is_a = [ObjectProperty, defines]
definesRole.domain = [Description]
definesRole.range = [Role]

isBearerDefinedIn.is_a = [ObjectProperty, isRoleDefinedIn]
isBearerDefinedIn.domain = [Role]
isBearerDefinedIn.range = [Affordance]

definesEventType.is_a = [ObjectProperty, defines]
definesEventType.domain = [Description]
definesEventType.range = [EventType]

isEventTypeDefinedIn.is_a = [ObjectProperty, isDefinedIn]
isEventTypeDefinedIn.domain = [EventType]
isEventTypeDefinedIn.range = [Description]

definesInput.is_a = [ObjectProperty, definesParticipant]
definesInput.domain = [Description]
definesInput.range = [Parameter | Role]

definesParticipant.is_a = [ObjectProperty, defines]
definesParticipant.domain = [Description]
definesParticipant.range = [Concept]

definesOutput.is_a = [ObjectProperty, definesParticipant]
definesOutput.domain = [Description]
definesOutput.range = [Parameter | Role]

definesParameter.is_a = [ObjectProperty, defines]
definesParameter.domain = [Description]
definesParameter.range = [Parameter]

isParameterDefinedIn.is_a = [ObjectProperty, isDefinedIn]
isParameterDefinedIn.domain = [Parameter]
isParameterDefinedIn.range = [Description]

isPerformerDefinedIn.is_a = [ObjectProperty, isRoleDefinedIn]
isPerformerDefinedIn.domain = [Role]
isPerformerDefinedIn.range = [Description]

definesProcess.is_a = [ObjectProperty, defines]
definesProcess.domain = [Description]
definesProcess.range = [ProcessType]

isProcessDefinedIn.is_a = [ObjectProperty, isDefinedIn]
isProcessDefinedIn.domain = [ProcessType]
isProcessDefinedIn.range = [Description]

isSetpointDefinedIn.is_a = [ObjectProperty, isParameterDefinedIn]
isSetpointDefinedIn.domain = [Setpoint]
isSetpointDefinedIn.range = [Description]

isTriggerDefinedIn.is_a = [ObjectProperty, isRoleDefinedIn]
isTriggerDefinedIn.domain = [Role]
isTriggerDefinedIn.range = [Affordance]

derivedFrom.is_a = [ObjectProperty, TransitiveProperty, associatedWith]
derivedFrom.domain = [InformationObject]
derivedFrom.range = [InformationObject]

isSourceFor.is_a = [ObjectProperty, TransitiveProperty, associatedWith]
isSourceFor.domain = [InformationObject]
isSourceFor.range = [InformationObject]

describesQuality.is_a = [ObjectProperty, describes]
describesQuality.domain = [Description]
describesQuality.range = [Quality]

describes.is_a = [ObjectProperty, associatedWith]
describes.domain = [Description]
describes.range = [Entity]

isQualityDescribedBy.is_a = [ObjectProperty, isDescribedBy]
isQualityDescribedBy.domain = [Quality]
isQualityDescribedBy.range = [Description]

directlyCauses.is_a = [ObjectProperty, causes]
directlyCauses.domain = [Event]
directlyCauses.range = [Event]

isDirectReactionTo.is_a = [ObjectProperty, isReactionTo]
isDirectReactionTo.domain = [Event]
isDirectReactionTo.range = [Event]

hasTerminalScene.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, hasTerminalState]
hasTerminalScene.domain = [StateTransition]
hasTerminalScene.range = [Scene]

includesEvent.is_a = [ObjectProperty, isSettingFor]
includesEvent.domain = [Situation]
includesEvent.range = [Event]

directlyDerivedFrom.is_a = [ObjectProperty, derivedFrom]

isDirectSourceFor.is_a = [ObjectProperty, isSourceFor]

encapsulates.is_a = [ObjectProperty, associatedWith]
encapsulates.domain = [OrderedElement]
encapsulates.range = [Entity]

encodes.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
encodes.domain = [InformationObject]
encodes.range = [InformationObject]

executesMotion.is_a = [ObjectProperty, isOccurrenceOf]
executesMotion.domain = [MotionProcess]
executesMotion.range = [Motion]

isOccurrenceOf.is_a = [ObjectProperty, isClassifiedBy]
isOccurrenceOf.domain = [Event]
isOccurrenceOf.range = [EventType]

isExecutedMotionIn.is_a = [ObjectProperty, isOccurringIn]
isExecutedMotionIn.domain = [Motion]
isExecutedMotionIn.range = [MotionProcess]

finishedBy.is_a = [ObjectProperty, TransitiveProperty, coOccurs]
finishedBy.domain = [Event]
finishedBy.range = [Event]

finishes.is_a = [ObjectProperty, TransitiveProperty, coOccurs]
finishes.domain = [Event]
finishes.range = [Event]

firstMember.is_a = [ObjectProperty, hasMember]
firstMember.domain = [Collection]
firstMember.range = [Entity]

hasMember.is_a = [ObjectProperty, associatedWith]
hasMember.domain = [Collection]
hasMember.range = [Entity]

givesMeaningTo.is_a = [ObjectProperty, associatedWith]
givesMeaningTo.domain = [System]
givesMeaningTo.range = [InformationObject]

isGivenMeaningBy.is_a = [ObjectProperty, associatedWith]
isGivenMeaningBy.domain = [InformationObject]
isGivenMeaningBy.range = [System]

hasAction.is_a = [ObjectProperty, hasConstituent]
hasAction.domain = [Action]
hasAction.range = [Action]

hasConstituent.is_a = [ObjectProperty, associatedWith]
hasConstituent.domain = [Entity]
hasConstituent.range = [Entity]

hasAlterationResult.is_a = [ObjectProperty, hasRegion]
hasAlterationResult.domain = [Action]
hasAlterationResult.range = [Region]

hasRegion.is_a = [ObjectProperty, associatedWith]
hasRegion.domain = [Entity]
hasRegion.range = [Region]

isAlterationResultOf.is_a = [ObjectProperty, isRegionFor]
isAlterationResultOf.domain = [Region]
isAlterationResultOf.range = [Action]

hasBinding.is_a = [ObjectProperty, hasPart]
hasBinding.domain = [Description]
hasBinding.range = [Binding]

hasPart.is_a = [ObjectProperty, TransitiveProperty, ReflexiveProperty, associatedWith]
hasPart.domain = [Entity]
hasPart.range = [Entity]

hasBindingFiller.is_a = [ObjectProperty, describes]
hasBindingFiller.domain = [Binding]
hasBindingFiller.range = [Entity]

hasBindingRole.is_a = [ObjectProperty, hasPart]
hasBindingRole.domain = [Binding]
hasBindingRole.range = [Parameter | Role]

hasChildLink.is_a = [ObjectProperty, hasConstituent]
hasChildLink.domain = [Joint]
hasChildLink.range = [PhysicalObject]

isChildLinkOf.is_a = [ObjectProperty, isConstituentOf]
isChildLinkOf.domain = [PhysicalObject]
isChildLinkOf.range = [Joint]

hasColor.is_a = [ObjectProperty, hasQuality]
hasColor.domain = [PhysicalObject]
hasColor.range = [Color]

hasQuality.is_a = [ObjectProperty, associatedWith]
hasQuality.domain = [Entity]
hasQuality.range = [Quality]

isColorOf.is_a = [ObjectProperty, isQualityOf]
isColorOf.domain = [Color]
isColorOf.range = [PhysicalObject]

hasDisposition.is_a = [ObjectProperty, hasQuality]
hasDisposition.domain = [Object]
hasDisposition.range = [Disposition]

isDispositionOf.is_a = [ObjectProperty, isQualityOf]
isDispositionOf.domain = [Disposition]
isDispositionOf.range = [Object]

hasEndLink.is_a = [ObjectProperty, hasLink]
hasEndLink.domain = [PhysicalObject]
hasEndLink.range = [PhysicalObject]

hasLink.is_a = [ObjectProperty, hasComponent]
hasLink.domain = [PhysicalObject]
hasLink.range = [PhysicalObject]

isEndLinkOf.is_a = [ObjectProperty, isLinkOf]
isEndLinkOf.domain = [PhysicalObject]
isEndLinkOf.range = [PhysicalObject]

hasExecutionState.is_a = [ObjectProperty, hasRegion]
hasExecutionState.domain = [Action]
hasExecutionState.range = [ExecutionStateRegion]

hasFeature.is_a = [ObjectProperty, hasConstituent]
hasFeature.domain = [PhysicalObject]
hasFeature.range = [Feature]

isFeatureOf.is_a = [ObjectProperty, isConstituentOf]
isFeatureOf.domain = [Feature]
isFeatureOf.range = [PhysicalObject]

hasFirstStep.is_a = [ObjectProperty, hasStep]
hasFirstStep.domain = [Workflow]
hasFirstStep.range = [Task]

hasStep.is_a = [ObjectProperty, definesTask]
hasStep.domain = [Workflow]
hasStep.range = [Task]

isFirstStepOf.is_a = [ObjectProperty, isStepOf]
isFirstStepOf.domain = [Task]
isFirstStepOf.range = [Workflow]

hasFrictionAttribute.is_a = [ObjectProperty, hasRegion]
hasFrictionAttribute.domain = [PhysicalObject]
hasFrictionAttribute.range = [FrictionAttribute]

hasGoal.is_a = [ObjectProperty, isDescribedBy]
hasGoal.domain = [Entity]
hasGoal.range = [Goal]

hasInitialScene.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, hasInitialState]
hasInitialScene.domain = [StateTransition]
hasInitialScene.range = [Scene]

hasInitialState.is_a = [ObjectProperty, includesSituation]
hasInitialState.domain = [Transition]
hasInitialState.range = [Situation]

isInitialSceneOf.is_a = [ObjectProperty, isInitialStateOf]
isInitialSceneOf.domain = [Scene]
isInitialSceneOf.range = [StateTransition]

hasInitialSituation.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, hasInitialState]
hasInitialSituation.domain = [SituationTransition]
hasInitialSituation.range = [Situation, Not(NonmanifestedSituation)]

isInitialSituationOf.is_a = [ObjectProperty, isInitialStateOf]
isInitialSituationOf.domain = [Situation]
isInitialSituationOf.range = [SituationTransition]

includesSituation.is_a = [ObjectProperty, isSettingFor]
includesSituation.domain = [Situation]
includesSituation.range = [Situation]

isInitialStateOf.is_a = [ObjectProperty, isSituationIncludedIn]
isInitialStateOf.domain = [Situation]
isInitialStateOf.range = [Transition]

hasInputParameter.is_a = [ObjectProperty, hasParameter]
hasInputParameter.domain = [EventType]
hasInputParameter.range = [Parameter]

hasParameter.is_a = [ObjectProperty, isRelatedToConcept]
hasParameter.domain = [Concept]
hasParameter.range = [Parameter]

isInputParameterFor.is_a = [ObjectProperty, isParameterFor]
isInputParameterFor.domain = [Parameter]
isInputParameterFor.range = [EventType]

hasJointLimit.is_a = [ObjectProperty, hasRegion]
hasJointLimit.domain = [Joint]
hasJointLimit.range = [JointLimit]

isJointLimitOf.is_a = [ObjectProperty, isRegionFor]
isJointLimitOf.domain = [JointLimit]
isJointLimitOf.range = [Joint]

hasJointState.is_a = [ObjectProperty, hasRegion]
hasJointState.domain = [Joint]
hasJointState.range = [JointState]

isJointStateOf.is_a = [ObjectProperty, isRegionFor]
isJointStateOf.domain = [JointState]
isJointStateOf.range = [Joint]

hasComponent.is_a = [ObjectProperty, AsymmetricProperty, hasProperPart]
hasComponent.domain = [Entity]
hasComponent.range = [Entity]

isLinkOf.is_a = [ObjectProperty, isComponentOf]
isLinkOf.domain = [PhysicalObject]
isLinkOf.range = [PhysicalObject]

hasLocalization.is_a = [ObjectProperty, hasQuality]
hasLocalization.domain = [PhysicalObject]
hasLocalization.range = [Localization]

isLocalizationOf.is_a = [ObjectProperty, isQualityOf]
isLocalizationOf.domain = [Localization]
isLocalizationOf.range = [PhysicalObject]

hasMassAttribute.is_a = [ObjectProperty, hasRegion]
hasMassAttribute.domain = [PhysicalObject]
hasMassAttribute.range = [MassAttribute]

isMassAttributeOf.is_a = [ObjectProperty, isRegionFor]
isMassAttributeOf.domain = [MassAttribute]
isMassAttributeOf.range = [PhysicalObject]

hasNetForce.is_a = [ObjectProperty, hasRegion]
hasNetForce.domain = [PhysicalObject]
hasNetForce.range = [NetForce]

isNetForceOf.is_a = [ObjectProperty, isRegionFor]
isNetForceOf.domain = [NetForce]
isNetForceOf.range = [PhysicalObject]

hasNextStep.is_a = [ObjectProperty, directlyPrecedes]
hasNextStep.domain = [Task]
hasNextStep.range = [Task]

directlyPrecedes.is_a = [ObjectProperty, precedes]
directlyPrecedes.domain = [Entity]
directlyPrecedes.range = [Entity]

hasPreviousStep.is_a = [ObjectProperty, directlyFollows]
hasPreviousStep.domain = [Task]
hasPreviousStep.range = [Task]

hasOutputParameter.is_a = [ObjectProperty, hasParameter]
hasOutputParameter.domain = [EventType]
hasOutputParameter.range = [Parameter]

isOutputParameterFor.is_a = [ObjectProperty, isParameterFor]
isOutputParameterFor.domain = [Parameter]
isOutputParameterFor.range = [EventType]

hasParentLink.is_a = [ObjectProperty, hasConstituent]
hasParentLink.domain = [Joint]
hasParentLink.range = [PhysicalObject]

isParentLinkOf.is_a = [ObjectProperty, isConstituentOf]
isParentLinkOf.domain = [PhysicalObject]
isParentLinkOf.range = [Joint]

hasPhase.is_a = [ObjectProperty, hasConstituent]
hasPhase.domain = [Action | Process]
hasPhase.range = [State | Process]

hasPhysicalComponent.is_a = [ObjectProperty, hasComponent]
hasPhysicalComponent.domain = [PhysicalObject]
hasPhysicalComponent.range = [PhysicalObject]

hasPredecessor.is_a = [ObjectProperty, hasPart]
hasPredecessor.domain = [Succedence]
hasPredecessor.range = [Workflow]

hasPreference.is_a = [ObjectProperty, hasQuality]
hasPreference.domain = [Agent]
hasPreference.range = [Preference]

isPreferenceOf.is_a = [ObjectProperty, isQualityOf]
isPreferenceOf.domain = [Preference]
isPreferenceOf.range = [Agent]

directlyFollows.is_a = [ObjectProperty, follows]
directlyFollows.domain = [Entity]
directlyFollows.range = [Entity]

hasProcessType.is_a = [ObjectProperty, isRelatedToConcept]
hasProcessType.domain = [Role]
hasProcessType.range = [ProcessType]

isRelatedToConcept.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
isRelatedToConcept.domain = [Concept]
isRelatedToConcept.range = [Concept]

isProcessTypeOf.is_a = [ObjectProperty, isRelatedToConcept]
isProcessTypeOf.domain = [ProcessType]
isProcessTypeOf.range = [Role]

hasQuale.is_a = [ObjectProperty, hasRegion]
hasQuale.domain = [Quality]
hasQuale.range = [Region]

isQualeOf.is_a = [ObjectProperty, isRegionFor]
isQualeOf.domain = [Region]
isQualeOf.range = [Quality]

hasRootLink.is_a = [ObjectProperty, hasLink]
hasRootLink.domain = [PhysicalObject]
hasRootLink.range = [PhysicalObject]

isRootLinkOf.is_a = [ObjectProperty, isLinkOf]
isRootLinkOf.domain = [PhysicalObject]
isRootLinkOf.range = [PhysicalObject]

hasShape.is_a = [ObjectProperty, hasQuality]
hasShape.domain = [PhysicalObject]
hasShape.range = [Shape]

isShapeOf.is_a = [ObjectProperty, isQualityOf]
isShapeOf.domain = [Shape]
isShapeOf.range = [PhysicalObject]

hasShapeRegion.is_a = [ObjectProperty, hasRegion]
hasShapeRegion.domain = [Shape | PhysicalObject]
hasShapeRegion.range = [ShapeRegion]

isShapeRegionOf.is_a = [ObjectProperty, isRegionFor]
isShapeRegionOf.domain = [ShapeRegion]
isShapeRegionOf.range = [Shape | PhysicalObject]

hasSoftwareAgent.is_a = [ObjectProperty, involvesAgent]
hasSoftwareAgent.domain = [Event]
hasSoftwareAgent.range = [SoftwareInstance]

involvesAgent.is_a = [ObjectProperty, hasParticipant]
involvesAgent.domain = [Event]
involvesAgent.range = [Agent]

hasSpaceRegion.is_a = [ObjectProperty, hasRegion]
hasSpaceRegion.domain = [Localization | ShapeRegion | PhysicalObject]
hasSpaceRegion.range = [SpaceRegion]

isSpaceRegionFor.is_a = [ObjectProperty, isRegionFor]
isSpaceRegionFor.domain = [SpaceRegion]
isSpaceRegionFor.range = [Localization | PhysicalObject]

hasStateType.is_a = [ObjectProperty, isRelatedToConcept]
hasStateType.domain = [Role]
hasStateType.range = [StateType]

isStateTypeOf.is_a = [ObjectProperty, isRelatedToConcept]
isStateTypeOf.domain = [StateType]
isStateTypeOf.range = [Role]

hasStatus.is_a = [ObjectProperty, hasQuality]

isStepOf.is_a = [ObjectProperty, isTaskDefinedIn]
isStepOf.domain = [Task]
isStepOf.range = [Workflow]

hasSuccedence.is_a = [ObjectProperty, hasPart]
hasSuccedence.domain = [Workflow]
hasSuccedence.range = [Succedence]

hasSuccessor.is_a = [ObjectProperty, hasPart]
hasSuccessor.domain = [Succedence]
hasSuccessor.range = [Workflow]

hasTask.is_a = [ObjectProperty, hasPart]
hasTask.domain = [Relation | Workflow]
hasTask.range = [Task]

hasTerminalState.is_a = [ObjectProperty, includesSituation]
hasTerminalState.domain = [Transition]
hasTerminalState.range = [Situation]

isTerminalSceneOf.is_a = [ObjectProperty, isTerminalStateOf]
isTerminalSceneOf.domain = [Scene]
isTerminalSceneOf.range = [StateTransition]

hasTerminalSituation.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, hasTerminalState]
hasTerminalSituation.domain = [SituationTransition]
hasTerminalSituation.range = [Situation, Not(NonmanifestedSituation)]

isTerminalSituationOf.is_a = [ObjectProperty, isTerminalStateOf]
isTerminalSituationOf.domain = [Situation]
isTerminalSituationOf.range = [SituationTransition]

isTerminalStateOf.is_a = [ObjectProperty, isSituationIncludedIn]
isTerminalStateOf.domain = [Situation]
isTerminalStateOf.range = [Transition]

includesConcept.is_a = [ObjectProperty, includesObject]
includesConcept.domain = [Situation]
includesConcept.range = [Concept]

includesObject.is_a = [ObjectProperty, isSettingFor]
includesObject.domain = [Situation]
includesObject.range = [Object]

isConceptIncludedIn.is_a = [ObjectProperty, isObjectIncludedIn]
isConceptIncludedIn.domain = [Concept]
isConceptIncludedIn.range = [Situation]

includesRecord.is_a = [ObjectProperty, isReferenceOf]
includesRecord.domain = [Event]
includesRecord.range = [InformationObject]

isReferenceOf.is_a = [ObjectProperty, associatedWith]
isReferenceOf.domain = [Entity]
isReferenceOf.range = [InformationObject]

isRecordIncludedBy.is_a = [ObjectProperty, isAbout]
isRecordIncludedBy.domain = [InformationObject]
isRecordIncludedBy.range = [Event]

isSettingFor.is_a = [ObjectProperty, associatedWith]
isSettingFor.domain = [Situation]
isSettingFor.range = [Entity]

isSituationIncludedIn.is_a = [ObjectProperty, hasSetting]
isSituationIncludedIn.domain = [Situation]
isSituationIncludedIn.range = [Situation]

involvesArtifact.is_a = [ObjectProperty, hasParticipant]
involvesArtifact.domain = [Event]
involvesArtifact.range = [PhysicalArtifact]

hasParticipant.is_a = [ObjectProperty, associatedWith]
hasParticipant.domain = [Event]
hasParticipant.range = [Object]

isArtifactInvolvedIn.is_a = [ObjectProperty, isParticipantIn]
isArtifactInvolvedIn.domain = [PhysicalArtifact]
isArtifactInvolvedIn.range = [Event]

involvesEffector.is_a = [ObjectProperty, hasParticipant]
involvesEffector.domain = [Event]
involvesEffector.range = [PhysicalEffector]

isEffectorInvolvedIn.is_a = [ObjectProperty, isParticipantIn]
isEffectorInvolvedIn.domain = [PhysicalEffector]
isEffectorInvolvedIn.range = [Event]

involvesPlace.is_a = [ObjectProperty, hasParticipant]
involvesPlace.domain = [Event]
involvesPlace.range = [PhysicalPlace]

isPlaceInvolvedIn.is_a = [ObjectProperty, isParticipantIn]
isPlaceInvolvedIn.domain = [PhysicalPlace]
isPlaceInvolvedIn.range = [Event]

isRegionFor.is_a = [ObjectProperty, associatedWith]
isRegionFor.domain = [Region]
isRegionFor.range = [Entity]

isAnsweredBy.is_a = [ObjectProperty, involvesAgent]
isAnsweredBy.domain = [Event]
isAnsweredBy.range = [Agent]

isParticipantIn.is_a = [ObjectProperty, associatedWith]
isParticipantIn.domain = [Object]
isParticipantIn.range = [Event]

isAskedBy.is_a = [ObjectProperty, involvesAgent]
isAskedBy.domain = [QueryingTask]
isAskedBy.range = [Agent]

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

isCreatedOutputOf.is_a = [ObjectProperty, isOutputRoleOf]
isCreatedOutputOf.domain = [Role]
isCreatedOutputOf.range = [Task]

isOutputRoleOf.is_a = [ObjectProperty, hasTask]
isOutputRoleOf.domain = [Role]
isOutputRoleOf.range = [Task]

isTaskOfCreatedRole.is_a = [ObjectProperty, isTaskOfOutputRole]
isTaskOfCreatedRole.domain = [Task]
isTaskOfCreatedRole.range = [Role]

isDefinedIn.is_a = [ObjectProperty, isConceptUsedIn]
isDefinedIn.domain = [Concept]
isDefinedIn.range = [Description]

isDepositOf.is_a = [ObjectProperty, isLocationOf]
isDepositOf.domain = [PhysicalObject]
isDepositOf.range = [PhysicalObject]

isOntopOf.is_a = [ObjectProperty, hasLocation]
isOntopOf.domain = [PhysicalObject]
isOntopOf.range = [PhysicalObject]

isDesignFor.is_a = [ObjectProperty, describes]
isDesignFor.domain = [Design]
isDesignFor.range = [Object]

isDesignedBy.is_a = [ObjectProperty, isDescribedBy]
isDesignedBy.domain = [Object]
isDesignedBy.range = [Design]

isRealizedBy.is_a = [ObjectProperty, associatedWith]
isRealizedBy.domain = [InformationObject]
isRealizedBy.range = [InformationRealization]

hasRole.is_a = [ObjectProperty, isClassifiedBy]
hasRole.domain = [Object]
hasRole.range = [Role]

isInputRoleOf.is_a = [ObjectProperty, hasTask]
isInputRoleOf.domain = [Role]
isInputRoleOf.range = [Task]

isRoleOf.is_a = [ObjectProperty, classifies]
isRoleOf.domain = [Role]
isRoleOf.range = [Object]

realizes.is_a = [ObjectProperty, associatedWith]
realizes.domain = [InformationRealization]
realizes.range = [InformationObject]

isOccurringIn.is_a = [ObjectProperty, classifies]
isOccurringIn.domain = [EventType]
isOccurringIn.range = [Event]

isExecutorDefinedIn.is_a = [ObjectProperty]

isParameterFor.is_a = [ObjectProperty, isRelatedToConcept]
isParameterFor.domain = [Parameter]
isParameterFor.range = [Concept]

hasTask.is_a = [ObjectProperty, isRelatedToConcept]
hasTask.domain = [Role]
hasTask.range = [Task]

isTaskOfInputRole.is_a = [ObjectProperty, isTaskOf]
isTaskOfInputRole.domain = [Task]
isTaskOfInputRole.range = [Role]

hasLocation.is_a = [ObjectProperty, associatedWith]
hasLocation.domain = [Entity]
hasLocation.range = [Entity]

isComponentOf.is_a = [ObjectProperty, AsymmetricProperty, isPropertPartOf]
isComponentOf.domain = [Entity]
isComponentOf.range = [Entity]

isLinkedTo.is_a = [ObjectProperty, SymmetricProperty, hasLocation]
isLinkedTo.domain = [PhysicalObject]
isLinkedTo.range = [PhysicalObject]

isMotionDescriptionFor.is_a = [ObjectProperty, describes]
isMotionDescriptionFor.domain = [MotionDescription]
isMotionDescriptionFor.range = [Motion]

isMovedByAgent.is_a = [ObjectProperty, interactsWith]
isMovedByAgent.domain = [PhysicalObject]
isMovedByAgent.range = [Agent]

movesObject.is_a = [ObjectProperty, interactsWith]
movesObject.domain = [Agent]
movesObject.range = [PhysicalObject]

isClassifiedBy.is_a = [ObjectProperty, associatedWith]
isClassifiedBy.domain = [Entity]
isClassifiedBy.range = [Concept]

classifies.is_a = [ObjectProperty, associatedWith]
classifies.domain = [Concept]
classifies.range = [Entity]

isOrderedBy.is_a = [ObjectProperty, associatedWith]
isOrderedBy.domain = [OrderedElement]
isOrderedBy.range = [Order]

orders.is_a = [ObjectProperty, associatedWith]
orders.domain = [Order]
orders.range = [OrderedElement]

isTaskOfOutputRole.is_a = [ObjectProperty, isTaskOf]
isTaskOfOutputRole.domain = [Task]
isTaskOfOutputRole.range = [Role]

isPerformedBy.is_a = [ObjectProperty, involvesAgent]
isPerformedBy.domain = [Action]
isPerformedBy.range = [Agent]

isPhysicallyContainedIn.is_a = [ObjectProperty, isLocationOf]
isPhysicallyContainedIn.domain = [Entity]
isPhysicallyContainedIn.range = [Entity]

isPlanFor.is_a = [ObjectProperty, describes]
isPlanFor.domain = [Plan]
isPlanFor.range = [Task]

isAbout.is_a = [ObjectProperty, associatedWith]
isAbout.domain = [InformationObject]
isAbout.range = [Entity]

isReplacedBy.is_a = [ObjectProperty, before]
isReplacedBy.domain = [State]
isReplacedBy.range = [State]

replaces.is_a = [ObjectProperty, after]
replaces.domain = [State]
replaces.range = [State]

hasSetting.is_a = [ObjectProperty, associatedWith]
hasSetting.domain = [Entity]
hasSetting.range = [Situation]

isTaskDefinedIn.is_a = [ObjectProperty, isDefinedIn]
isTaskDefinedIn.domain = [Task]
isTaskDefinedIn.range = [Description]

isSupportedBy.is_a = [ObjectProperty, interactsWith, hasCommonBoundary]
isSupportedBy.domain = [Entity]
isSupportedBy.range = [Entity]

hasCommonBoundary.is_a = [ObjectProperty, SymmetricProperty, associatedWith]
hasCommonBoundary.domain = [Entity]
hasCommonBoundary.range = [Entity]

supports.is_a = [ObjectProperty, interactsWith, hasCommonBoundary]
supports.domain = [Entity]
supports.range = [Entity]

isTaskOf.is_a = [ObjectProperty, isRelatedToConcept]
isTaskOf.domain = [Task]
isTaskOf.range = [Role]

isTerminatedBy.is_a = [ObjectProperty, associatedWith]
isTerminatedBy.domain = [Event]
isTerminatedBy.range = [Event]

terminates.is_a = [ObjectProperty, associatedWith]
terminates.domain = [Event]
terminates.range = [Event]

meets.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, directlyPrecedes]
meets.domain = [Entity]
meets.range = [Entity]

metBy.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, directlyFollows]
metBy.domain = [Entity]
metBy.range = [Entity]

overlappedBy.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, overlaps]
overlappedBy.domain = [Entity]
overlappedBy.range = [Entity]

overlappedOn.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, overlaps]
overlappedOn.domain = [Entity]
overlappedOn.range = [Entity]

simultaneous.is_a = [ObjectProperty, SymmetricProperty, TransitiveProperty, coOccurs]
simultaneous.domain = [Event]
simultaneous.range = [Event]

startedBy.is_a = [ObjectProperty, TransitiveProperty, coOccurs]
startedBy.domain = [Event]
startedBy.range = [Event]

starts.is_a = [ObjectProperty, TransitiveProperty, coOccurs]
starts.domain = [Event]
starts.range = [Event]

transitionsBack.is_a = [ObjectProperty, transitionsFrom, transitionsTo]
transitionsBack.domain = [Transient]
transitionsBack.range = [Object]

transitionsFrom.is_a = [ObjectProperty, hasCommonBoundary]
transitionsFrom.domain = [Transient]
transitionsFrom.range = [Object]

transitionsTo.is_a = [ObjectProperty, hasCommonBoundary]
transitionsTo.domain = [Transient]
transitionsTo.range = [Object]

hasExpectedTerminalSituation.is_a = [ObjectProperty, hasTerminalSituation, hasPostcondition]
hasExpectedTerminalSituation.domain = [SituationTransition]
hasExpectedTerminalSituation.range = [Situation]

hasPostcondition.is_a = [ObjectProperty, directlyPrecedes]
hasPostcondition.domain = [Event | Situation]
hasPostcondition.range = [Event | Situation]

hasRequiredInitialSituation.is_a = [ObjectProperty, hasInitialSituation, hasPrecondition]
hasRequiredInitialSituation.domain = [SituationTransition]
hasRequiredInitialSituation.range = [Situation]

hasPrecondition.is_a = [ObjectProperty, directlyFollows]
hasPrecondition.domain = [Event | Situation]
hasPrecondition.range = [Event | Situation]

manifestsIn.is_a = [ObjectProperty, includesEvent]

preventedBy.is_a = [ObjectProperty, hasSetting]
preventedBy.domain = [Situation]
preventedBy.range = [Situation]

prevents.is_a = [ObjectProperty, isSettingFor]
prevents.domain = [Situation]
prevents.range = [Situation]

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




