from .dependencies import *
from .classes import *
from .individuals import *


Location.is_a = [SpatioTemporalRole]

Room.is_a = [PhysicalPlace]

Entity.is_a = [Thing]

Appliance.is_a = [DesignedArtifact]

Deposition.is_a = [Disposition, affords_bearer.only(Deposit), affords_trigger.only(DepositedObject)]

DesignedComponent.is_a = [FunctionalPart, DesignedArtifact, has_disposition.some(Connectivity)]

DesignedContainer.is_a = [DesignedArtifact, has_disposition.some(Containment)]

DesignedFurniture.is_a = [DesignedArtifact]

DesignedTool.is_a = [DesignedArtifact]

Disposition.is_a = [Extrinsic, is_described_by.exactly(1, Affordance), is_described_by.exactly(1, And([Affordance, defines_bearer.exactly(1, Role), defines_event_type.exactly(1, EventType), defines_trigger.exactly(1, Role)]))]

Fluid.is_a = [Substance]

Intrinsic.is_a = [PhysicalQuality]

Preference.is_a = [SocialQuality, is_preference_of.only(Agent)]

SocialObject.is_a = [Object, is_expressed_by.some(InformationObject), has_part.only(SocialObject)]

Action.is_a = [Event, has_participant.some(Agent), executes_task.min(1, Thing)]

State.is_a = [Event]

DesignedArtifact.is_a = [PhysicalArtifact, is_described_by.some(Design)]

Substance.is_a = [PhysicalBody]

Collection.is_a = [SocialObject, has_part.only(Collection)]

Plan.is_a = [Description, has_component.some(Goal)]

Affordance.is_a = [Relation, describes.only(Disposition), defines_bearer.exactly(1, Role), defines_trigger.exactly(1, Role), defines_task.exactly(1, Task)]

Concept.is_a = [SocialObject, is_defined_in.some(Description), has_part.only(Concept)]

Task.is_a = [EventType, has_part.only(Task), is_executed_in.only(Action), is_task_defined_in.only(Description), is_task_of.only(Role)]

Role.is_a = [Concept, classifies.only(Object), has_part.only(Role)]

Setpoint.is_a = [Parameter]

Answer.is_a = [Message]

Message.is_a = [Item, has_task.some(CommunicationTask), classifies.only(InformationRealization)]

PhysicalObject.is_a = [Object, has_part.only(PhysicalObject)]

Description.is_a = [SocialObject]

EventType.is_a = [Concept, classifies.only(Event)]

Parameter.is_a = [Concept, classifies.only(Region), has_part.only(Parameter)]

ProcessType.is_a = [EventType, classifies.only(Process), has_part.only(ProcessType), is_defined_in.only(Description)]

Quality.is_a = [Entity, has_region.some(Region), is_quality_of.some(Entity), has_constituent.only(Quality), has_part.only(Quality)]

Event.is_a = [Entity, has_participant.some(Object), has_time_interval.some(TimeInterval), has_constituent.only(Event), has_part.only(Event)]

OrderedElement.is_a = [Singleton, follows.only(OrderedElement), precedes.only(OrderedElement), is_ordered_by.exactly(1, Order)]

InformationObject.is_a = [InformationEntity, SocialObject]

MotionProcess.is_a = [PhysicsProcess]

Motion.is_a = [ProcessType, is_executed_motion_in.only(MotionProcess)]

System.is_a = [SocialObject]

Region.is_a = [Abstract, has_constituent.only(Region), has_part.only(Region), overlaps.only(Region), precedes.only(Region)]

Binding.is_a = [Relation, Or([CounterfactualBinding, FactualBinding]), Or([RoleFillerBinding, RoleRoleBinding]), Inverse(has_binding).some(Description), has_binding_filler.exactly(1, Entity), has_binding_role.exactly(1, Or([Parameter, Role]))]

Joint.is_a = [PhysicalBody, has_child_link.exactly(1, PhysicalObject), has_parent_link.exactly(1, PhysicalObject), has_joint_state.max(1, JointState)]
Joint.equivalent_to = [Or([FixedJoint, MovableJoint])]

Color.is_a = [Extrinsic, has_region.some(ColorRegion), has_region.only(ColorRegion)]

Object.is_a = [Entity, has_location.some(Entity), is_participant_in.some(Event), has_constituent.only(Object), has_part.only(Object), is_classified_by.only(Role)]

ExecutionStateRegion.is_a = [Region]
ExecutionStateRegion.equivalent_to = [OneOf([ExecutionState_Active, ExecutionState_Cancelled, ExecutionState_Failed, ExecutionState_Paused, ExecutionState_Pending, ExecutionState_Succeeded])]

Feature.is_a = [Object, has_part.only(Feature), is_feature_of.exactly(1, PhysicalObject)]

Workflow.is_a = [Plan, defines_role.some(Role), defines_task.some(Task)]

FrictionAttribute.is_a = [PhysicalAttribute, has_friction_value.exactly(1, float)]

SituationTransition.is_a = [Transition]

Situation.is_a = [Entity, satisfies.some(Description)]

NonmanifestedSituation.is_a = [Situation]
NonmanifestedSituation.equivalent_to = [And([Situation, manifests_in.exactly(0, Event)])]

JointLimit.is_a = [PhysicalAttribute]

JointState.is_a = [PhysicalAttribute, has_joint_position.exactly(1, float), has_joint_velocity.exactly(1, float), has_joint_effort.max(1, float)]

Localization.is_a = [Extrinsic, has_region.some(SpaceRegion), has_region.only(SpaceRegion)]

MassAttribute.is_a = [PhysicalAttribute, has_mass_value.exactly(1, float)]

NetForce.is_a = [ForceAttribute]

Succedence.is_a = [Relation, Inverse(has_succedence).some(Description), has_predecessor.exactly(1, TaskInvocation), has_successor.exactly(1, TaskInvocation)]

Agent.is_a = [Object]

Shape.is_a = [Intrinsic, has_region.some(ShapeRegion), has_region.only(ShapeRegion)]

ShapeRegion.is_a = [Region, has_space_region.max(1, SixDPose)]

SoftwareInstance.is_a = [Thing, And([SocialAgent, acts_for.some(Agent)]), is_designed_by.some(Software)]

SpaceRegion.is_a = [Region]

StateType.is_a = [EventType, classifies.only(State), has_part.only(StateType)]

Relation.is_a = [Description]

PhysicalEffector.is_a = [FunctionalPart, Inverse(has_part).some(PhysicalAgent)]

QueryingTask.is_a = [IllocutionaryTask, is_task_of.some(Query), classifies.only(has_participant.some(InterrogativeClause))]

Design.is_a = [Description]

MotionDescription.is_a = [ProcessFlow]

Order.is_a = [FormalEntity, orders.some(OrderedElement)]

Transient.is_a = [Object, transitions_from.some(Object)]

ColorRegion.is_a = [PhysicalAttribute, is_region_for.only(Color)]

ForceAttribute.is_a = [PhysicalAttribute, has_force_value.exactly(1, float)]

PhysicalAttribute.is_a = [Region, is_region_for.only(PhysicalObject)]

APISpecification.is_a = [InterfaceSpecification]

InterfaceSpecification.is_a = [Design]

AbductiveReasoning.is_a = [Reasoning]

Reasoning.is_a = [DerivingInformation]

Accessor.is_a = [Instrument]

Instrument.is_a = [ResourceRole]

Accident.is_a = [Event]

ActionExecutionPlan.is_a = [Plan, defines_task.only(has_parameter.some(Status))]

Actuating.is_a = [PhysicalTask]

PhysicalTask.is_a = [Task, classifies.only(And([Action, (Or([has_participant.some(PhysicalAgent), has_participant.some(PhysicalObject)]))]))]

AestheticDesign.is_a = [Design]

AgentRole.is_a = [CausativeRole, classifies.only(Agent)]

CausativeRole.is_a = [EventAdjacentRole]

Agonist.is_a = [Patient]

Patient.is_a = [EventAdjacentRole]

Algorithm.is_a = [Plan]

Alteration.is_a = [ProcessType]

AlterativeInteraction.is_a = [ForceInteraction]

ForceInteraction.is_a = [ProcessType, is_process_type_of.some(Agonist), is_process_type_of.some(Antagonist)]
ForceInteraction.equivalent_to = [Or([AlterativeInteraction, PreservativeInteraction])]

PreservativeInteraction.is_a = [ForceInteraction]

AlteredObject.is_a = [Patient]

Amateurish.is_a = [DexterityDiagnosis]

AnsweringTask.is_a = [Thing]
AnsweringTask.equivalent_to = [And([IllocutionaryTask, classifies.only(is_reaction_to.only(is_classified_by.some(CommandingTask)))]), is_task_of.some(Answer)]

CommandingTask.is_a = [IllocutionaryTask, classifies.only(has_participant.some(ImperativeClause))]

IllocutionaryTask.is_a = [Thing, And([CommunicationTask, classifies.only(has_participant.only(Or([Agent, SocialObject])))])]

Antagonist.is_a = [Patient]

Approaching.is_a = [Locomotion]

Locomotion.is_a = [BodyMovement, DirectedMotion]

ArchiveFile.is_a = [DigitalFile]
ArchiveFile.equivalent_to = [And([DigitalFile, realizes.some(ArchiveText)])]

ArchiveText.is_a = [Thing]
ArchiveText.equivalent_to = [And([StructuredText, expresses.some(FileConfiguration)]), is_given_meaning_by.some(ArchiveFormat)]

DigitalFile.is_a = [InformationRealization, realizes.some(StructuredText), has_name_string.some(str)]

ArchiveFormat.is_a = [FileFormat, gives_meaning_to.only(ArchiveText)]

FileFormat.is_a = [ComputerLanguage]

FileConfiguration.is_a = [Thing]
FileConfiguration.equivalent_to = [And([Configuration, has_member.only(DigitalFile)])]

StructuredText.is_a = [Thing]
StructuredText.equivalent_to = [And([Text, is_given_meaning_by.some(FormalLanguage)])]

AreaSurveying.is_a = [Perceiving]

Perceiving.is_a = [Thing, And([InformationAcquisition, PhysicalTask])]

Arm.is_a = [Limb]

Limb.is_a = [PhysicalEffector]
Limb.equivalent_to = [Or([Arm, Leg])]

Arranging.is_a = [Constructing]

Constructing.is_a = [PhysicalTask]

ArtificialAgent.is_a = [PhysicalAgent]

PhysicalAgent.is_a = [Agent, PhysicalObject]

Assembling.is_a = [Constructing]

AssertionTask.is_a = [IllocutionaryTask, classifies.only(has_participant.some(DeclarativeClause))]

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

BehavioralDiagnosis.is_a = [Diagnosis, has_constituent.some(Goal)]

BeneficiaryRole.is_a = [GoalRole]

GoalRole.is_a = [EventAdjacentRole]

CounterfactualBinding.is_a = [Binding]

FactualBinding.is_a = [Binding]

RoleFillerBinding.is_a = [Binding]

RoleRoleBinding.is_a = [Binding, has_binding_filler.only(Or([Parameter, Role]))]

Blockage.is_a = [Disposition, affords_bearer.only(Barrier), affords_trigger.only(BlockedObject)]

BlockedObject.is_a = [Patient]

BodyMovement.is_a = [Motion, classifies.only(Or([has_participant.some(PhysicalAgent), has_participant.some(And([PhysicalObject, is_part_of.some(PhysicalAgent)]))]))]

Boiling.is_a = [Vaporizing]

Vaporizing.is_a = [PhaseTransition, is_process_type_of.some(And([AlteredObject, classifies.only(Substance)]))]

BoxShape.is_a = [ShapeRegion, has_height.exactly(1, float), has_length.exactly(1, float), has_width.exactly(1, float)]

CanCut.is_a = [Variability, affords_bearer.only(Cutter), affords_trigger.only(CutObject)]

Variability.is_a = [Disposition, affords_bearer.only(Tool), affords_trigger.only(AlteredObject)]

Cutter.is_a = [Tool]

CutObject.is_a = [AlteredObject]

Capability.is_a = [Disposition, is_described_by.exactly(1, And([Affordance, defines_bearer.exactly(1, Role), defines_trigger.exactly(1, Role), defines_task.exactly(1, Task)]))]

Capacity.is_a = [Intrinsic]

Catching.is_a = [Actuating]

PickingUp.is_a = [Manipulating]

CausalEventRole.is_a = [CausativeRole]

EventAdjacentRole.is_a = [Role]

CausedMotionTheory.is_a = [ImageSchemaTheory, defines.some(CausalEventRole), defines.some(Instrument), defines.some(Patient), defines.some(PerformerRole), has_part.some(SourcePathGoalTheory)]

ImageSchemaTheory.is_a = [SchematicTheory]

PerformerRole.is_a = [CausativeRole, classifies.only(Agent)]

SourcePathGoalTheory.is_a = [ImageSchemaTheory, defines.some(Destination), defines.some(Origin), defines.some(PathRole)]

RoomSurface.is_a = [Surface]

Channel.is_a = [PathRole, has_task.some(CommunicationTask)]

PathRole.is_a = [SpatioTemporalRole]

CommunicationTask.is_a = [Task, And([is_task_of.some(Channel), is_task_of.some(Message), is_task_of.some(Receiver), is_task_of.some(Sender)]), classifies.only(has_participant.only(Or([Agent, SocialObject])))]

CheckingObjectPresence.is_a = [Perceiving]

ChemicalProcess.is_a = [Process]

Process.is_a = [Event]

Choice.is_a = [ResultRole]

ResultRole.is_a = [GoalRole]

CircularCylinder.is_a = [CylinderShape, has_radius.exactly(1, float)]

CylinderShape.is_a = [ShapeRegion, has_radius.some(float), has_length.exactly(1, float)]

Classifier.is_a = [StatisticalReasoner]

StatisticalReasoner.is_a = [Reasoner]

ClausalObject.is_a = [Phrase]

Phrase.is_a = [LinguisticObject]

Clean.is_a = [CleanlinessRegion]

CleanlinessRegion.is_a = [Region]

Cleaning.is_a = [ModifyingPhysicalObject, is_task_afforded_by.some(Purification)]

ModifyingPhysicalObject.is_a = [PhysicalTask]

Purification.is_a = [Variability, affords_setpoint.only(classifies.only(And([Clean, is_region_for.only(Cleanliness)]))), affords_trigger.only(classifies.only(PhysicalObject))]

Cleanliness.is_a = [SocialQuality, has_region.some(CleanlinessRegion), has_region.only(CleanlinessRegion)]

SocialQuality.is_a = [Quality]

ClientServerSpecification.is_a = [InterfaceSpecification, And([defines_role.some(ClientRole), defines_role.some(ServerRole)])]

ClientRole.is_a = [InterfaceComponentRole, is_defined_in.some(ClientServerSpecification)]

ServerRole.is_a = [InterfaceComponentRole, is_defined_in.some(ClientServerSpecification)]

InterfaceComponentRole.is_a = [SoftwareRole]
InterfaceComponentRole.equivalent_to = [And([SoftwareRole, is_defined_in.only(InterfaceSpecification)])]

Closing.is_a = [Actuating]

Delivering.is_a = [Actuating, is_task_afforded_by.some(Shifting)]

Fetching.is_a = [PhysicalAcquiring]

Lifting.is_a = [Actuating]

Opening.is_a = [Actuating]

Pulling.is_a = [Actuating]

Pushing.is_a = [Actuating]

Squeezing.is_a = [Actuating]

Linkage.is_a = [Connectivity, affords_bearer.only(LinkedObject), affords_trigger.only(LinkedObject)]

Clumsiness.is_a = [Amateurish]

CognitiveAgent.is_a = [Agent]

SubCognitiveAgent.is_a = [Agent]

Collision.is_a = [ProcessType]

Extrinsic.is_a = [PhysicalQuality]

ImperativeClause.is_a = [ClausalObject, expresses.some(StateTransition), expresses.only(StateTransition)]

CommitedObject.is_a = [ConnectedObject]

ConnectedObject.is_a = [Patient]

CommunicationAction.is_a = [Action, has_participant.some(LinguisticObject)]

LinguisticObject.is_a = [InformationObject]

CommunicationReport.is_a = [CommunicationTask]

Receiver.is_a = [ExperiencerRole, has_task.some(CommunicationTask)]

Sender.is_a = [PerformerRole, has_task.some(CommunicationTask)]

CommunicationTopic.is_a = [ResourceRole, classifies.only(And([SocialObject, is_participant_in.some(is_classified_by.some(CommunicationTask))]))]

ResourceRole.is_a = [EventAdjacentRole]

Composing.is_a = [Variability, affords_trigger.only(ConnectedObject)]

ComputerLanguage.is_a = [FormalLanguage]

FormalLanguage.is_a = [Language, gives_meaning_to.only(StructuredText)]

ComputerProgram.is_a = [Thing]
ComputerProgram.equivalent_to = [And([StructuredText, is_given_meaning_by.some(ProgrammingLanguage)]), And([StructuredText, expresses.some(Algorithm)])]

ProgrammingLanguage.is_a = [ComputerLanguage, gives_meaning_to.only(ComputerProgram)]

Conclusion.is_a = [CreatedObject, Knowledge]

CreatedObject.is_a = [Patient]

Knowledge.is_a = [Item]

ConditionalSuccedence.is_a = [Succedence, has_binding.only(CounterfactualBinding)]

Configuration.is_a = [Description, describes.some(StateType)]

Connectivity.is_a = [Disposition, affords_bearer.only(ConnectedObject), affords_trigger.only(ConnectedObject)]

ContactState.is_a = [StateType, classifies.only(has_participant.min(2, PhysicalObject))]

Container.is_a = [Restrictor]

Containment.is_a = [Disposition, affords_bearer.only(Container), affords_trigger.only(IncludedObject), is_disposition_of.only(has_quality.some(Capacity))]

IncludedObject.is_a = [Patient]

ContainmentState.is_a = [FunctionalControl, is_state_type_of.some(And([Container, classifies.only(has_disposition.some(Containment))]))]

FunctionalControl.is_a = [StateType, is_state_type_of.some(Item), is_state_type_of.some(Restrictor)]

ContainmentTheory.is_a = [ControlTheory]

ControlTheory.is_a = [FunctionalSpatialSchemaTheory]

ContinuousJoint.is_a = [HingeJoint]

HingeJoint.is_a = [MovableJoint]

FunctionalSpatialSchemaTheory.is_a = [ImageSchemaTheory, defines.some(LocatumRole), defines.some(RelatumRole)]

Cover.is_a = [Barrier]

Coverage.is_a = [Blockage, affords_bearer.only(Cover), affords_trigger.only(CoveredObject)]

CoveredObject.is_a = [BlockedObject]

CoverageTheory.is_a = [FunctionalSpatialSchemaTheory]

CoveringTheory.is_a = [ExecutableSchematicTheory, defines.some(Instrument), defines.exactly(1, Patient)]

ExecutableSchematicTheory.is_a = [SchematicTheory, defines.some(PerformerRole)]

CrackingTheory.is_a = [ExecutableSchematicTheory, defines.some(Instrument), defines.some(Patient)]

Creation.is_a = [ProcessType, is_process_type_of.some(CreatedObject)]

Insertion.is_a = [Enclosing, affords_trigger.only(InsertedObject)]

Cuttability.is_a = [Disposition, affords_bearer.only(CutObject), affords_trigger.only(Cutter)]

Tool.is_a = [Instrument]

Cutting.is_a = [ModifyingPhysicalObject]

Shaping.is_a = [Variability, affords_setpoint.only(classifies.only(Shape)), affords_trigger.only(ShapedObject)]

Database.is_a = [SoftwareRole]

SoftwareRole.is_a = [Role, classifies.only(Software)]

Deciding.is_a = [DerivingInformation]

DerivingInformation.is_a = [InformationAcquisition, And([is_task_of_input_role.some(Premise), is_task_of_output_role.some(Conclusion)])]

DeductiveReasoning.is_a = [Reasoning]

Deformation.is_a = [Alteration, is_process_type_of.exactly(1, ShapedObject)]

ShapedObject.is_a = [AlteredObject, is_trigger_defined_in.some(describes_quality.only(Shape)), classifies.only(has_quality.some(Shape))]

FluidFlow.is_a = [Motion, is_process_type_of.some(MovedObject)]

Shifting.is_a = [Variability, affords_setpoint.only(classifies.only(Localization)), affords_trigger.only(MovedObject)]

DependentPlace.is_a = [Feature]

Deposit.is_a = [Instrument]

DepositedObject.is_a = [Patient]

InformationAcquisition.is_a = [Thing]
InformationAcquisition.equivalent_to = [And([MentalTask, is_task_of_output_role.some(Knowledge)])]

Premise.is_a = [Knowledge]

FunctionalPart.is_a = [PhysicalBody, is_component_of.only(Or([Agent, DesignedArtifact]))]

Graspability.is_a = [Disposition]

Destination.is_a = [Location]

DestroyedObject.is_a = [Patient]

Destruction.is_a = [ProcessType, is_process_type_of.some(DestroyedObject)]

DetectedObject.is_a = [Patient]

DeviceState.is_a = [Intrinsic]

DeviceStateRange.is_a = [Region]
DeviceStateRange.equivalent_to = [Or([DeviceTurnedOff, DeviceTurnedOn])]

DeviceTurnedOff.is_a = [DeviceStateRange]

DeviceTurnedOn.is_a = [DeviceStateRange]

DexterityDiagnosis.is_a = [BehavioralDiagnosis]

Dicing.is_a = [Cutting]

InformationRealization.is_a = [InformationEntity, Or([Event, PhysicalObject, Quality]), realizes.some(InformationObject), realizes_self_information.has_self()]

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

Enclosing.is_a = [Containment, affords_trigger.only(EnclosedObject)]

EndEffectorPositioning.is_a = [Manipulating]

Manipulating.is_a = [PhysicalTask, classifies.only(PhysicalAction)]

Episode.is_a = [Situation]

ExcludedObject.is_a = [Patient]

ExecutableFile.is_a = [Thing]
ExecutableFile.equivalent_to = [And([DigitalFile, realizes.some(ExecutableCode)])]

ExecutableCode.is_a = [ComputerProgram]
ExecutableCode.equivalent_to = [And([ComputerProgram, is_given_meaning_by.some(ExecutableFormat)])]

ExecutableFormat.is_a = [FileFormat, gives_meaning_to.only(ExecutableCode)]

SchematicTheory.is_a = [Theory]

ExecutableSoftware.is_a = [Thing]
ExecutableSoftware.equivalent_to = [And([Software, has_member.some(ExecutableFile)])]

Software.is_a = [Design, describes.some(SoftwareConfiguration), describes.only(Or([SoftwareInstance, SoftwareConfiguration]))]

RelationAdjacentRole.is_a = [Role]

ExperiencerRole.is_a = [PerformerRole]

ExtractedObject.is_a = [Patient]

PhysicalQuality.is_a = [Quality, is_quality_of.exactly(1, PhysicalObject)]

FailedAttempt.is_a = [Unsuccessfulness]

FaultySoftware.is_a = [SoftwareDiagnosis]

SoftwareDiagnosis.is_a = [TechnicalDiagnosis]

PhysicalAcquiring.is_a = [ModifyingPhysicalObject]

Configuration.is_a = [Collection]

Finger.is_a = [Limb, is_part_of.some(Hand)]

Hand.is_a = [PrehensileEffector]

FixedJoint.is_a = [Joint, has_joint_state.exactly(0, Entity)]

MovableJoint.is_a = [Joint, has_joint_state.exactly(1, JointState)]

Flipping.is_a = [Actuating, is_task_afforded_by.some(Shifting)]

FloatingJoint.is_a = [MovableJoint]

MovedObject.is_a = [AlteredObject, is_trigger_defined_in.some(describes_quality.only(Localization)), classifies.only(has_quality.some(Localization))]

Focusing.is_a = [AttentionShift]

Foolishness.is_a = [Amateurish]

ForgettingIncorrectInformation.is_a = [InformationDismissal]

InformationDismissal.is_a = [MentalTask, is_task_of_input_role.some(And([ExcludedObject, Knowledge]))]

ForgettingIrrelevantInformation.is_a = [InformationDismissal]

Language.is_a = [System, gives_meaning_to.only(Text)]

Item.is_a = [Patient]

FunctionalDesign.is_a = [Design]

FunctionalDiagnosis.is_a = [Diagnosis]

PhysicalBody.is_a = [PhysicalObject]

LocatumRole.is_a = [SpatialRelationRole, classifies.only(PhysicalObject)]

RelatumRole.is_a = [SpatialRelationRole, classifies.only(PhysicalObject)]

GetTaskParameter.is_a = [Planning]

Planning.is_a = [Deciding, is_task_of.some(classifies.some(Plan))]

GraphDatabase.is_a = [Database]

GraphQueryLanguage.is_a = [QueryLanguage]

QueryLanguage.is_a = [ComputerLanguage]

GraspTransfer.is_a = [Grasping]

Grasping.is_a = [Manipulating]

Releasing.is_a = [Manipulating]

GraspingMotion.is_a = [PrehensileMotion]
GraspingMotion.equivalent_to = [Or([IntermediateGrasp, PowerGrasp, PrecisionGrasp])]

IntermediateGrasp.is_a = [GraspingMotion]

PowerGrasp.is_a = [GraspingMotion]

PrecisionGrasp.is_a = [GraspingMotion]

PrehensileMotion.is_a = [DirectedMotion, is_process_type_of.some(And([MovedObject, classifies.only(PrehensileEffector)]))]
PrehensileMotion.equivalent_to = [Or([GraspingMotion, ReleasingMotion])]

ReleasingMotion.is_a = [PrehensileMotion]

GreenColor.is_a = [ColorRegion]

Gripper.is_a = [PrehensileEffector]

PrehensileEffector.is_a = [PhysicalEffector]

HardwareDiagnosis.is_a = [TechnicalDiagnosis]

HasQualityRegion.is_a = [Relation, has_quality.exactly(1, Quality), has_region.exactly(1, Region)]

Head.is_a = [FunctionalPart]

HeadMovement.is_a = [BodyMovement]

HeadTurning.is_a = [HeadMovement]

Holding.is_a = [Manipulating]

HostRole.is_a = [InterfaceComponentRole, is_defined_in.some(PluginSpecification)]

PluginSpecification.is_a = [APISpecification, And([defines_role.some(HostRole), defines_role.some(PluginRole)])]

HumanreadableProgrammingLanguage.is_a = [ProgrammingLanguage, gives_meaning_to.only(SourceCode)]

SourceCode.is_a = [ComputerProgram]
SourceCode.equivalent_to = [And([ComputerProgram, is_given_meaning_by.some(HumanreadableProgrammingLanguage)])]

HumanActivityRecording.is_a = [RecordedEpisode]

Imagining.is_a = [InformationAcquisition]

Impediment.is_a = [Blockage, affords_bearer.only(Obstacle), affords_trigger.only(RestrictedObject)]

Obstacle.is_a = [Barrier]

RestrictedObject.is_a = [BlockedObject]

StateTransition.is_a = [Transition, has_initial_scene.some(Scene), has_terminal_scene.some(Scene), includes_event.some(Action), satisfies.some(ImageSchemaTheory)]

Inability.is_a = [Unsuccessfulness]

IncompatibleSoftware.is_a = [SoftwareDiagnosis]

InductiveReasoning.is_a = [Reasoning]

Infeasibility.is_a = [Unsuccessfulness]

InferenceRules.is_a = [Knowledge]

InformationRetrieval.is_a = [InformationAcquisition, is_task_of_output_role.some(Knowledge)]

InformationStorage.is_a = [MentalTask, is_task_of_input_role.some(And([Knowledge, StoredObject]))]

StoredObject.is_a = [EnclosedObject]

InsertedObject.is_a = [EnclosedObject]

Instructions.is_a = [Item]

Interpreting.is_a = [DerivingInformation]

InterrogativeClause.is_a = [ClausalObject]

Introspecting.is_a = [InformationAcquisition]

KineticFrictionAttribute.is_a = [FrictionAttribute]

KinoDynamicData.is_a = [InformationObject, is_about.only(PhysicalObject)]

KnowledgeRepresentationLanguage.is_a = [ComputerLanguage]

Labeling.is_a = [Interpreting]

Text.is_a = [Thing]
Text.equivalent_to = [And([InformationObject, is_given_meaning_by.some(Language)])]

Leaning.is_a = [PosturalMoving]

PosturalMoving.is_a = [BodyMovement]

Learning.is_a = [InformationStorage]

Leg.is_a = [Limb]

LimbMotion.is_a = [DirectedMotion, is_process_type_of.some(And([MovedObject, classifies.only(Limb)]))]

LinkedObject.is_a = [ConnectedObject]

LinkageState.is_a = [StateType, is_state_type_of.some(LinkedObject)]

SpatioTemporalRole.is_a = [EventAdjacentRole]

SpatialRelationRole.is_a = [RelationAdjacentRole]

LocutionaryAction.is_a = [Thing]
LocutionaryAction.equivalent_to = [And([CommunicationAction, has_participant.some(Agent)])]

LookingAt.is_a = [PhysicalTask]

LookingFor.is_a = [Perceiving]

Lowering.is_a = [Actuating]

PhysicalAction.is_a = [Action]

MarkupLanguage.is_a = [ComputerLanguage]

Masterful.is_a = [DexterityDiagnosis]

Material.is_a = [Intrinsic]

MedicalDiagnosis.is_a = [FunctionalDiagnosis, has_constituent.some(Organism)]

Memorizing.is_a = [Learning]

MentalAction.is_a = [Action, has_participant.only(Or([Agent, SocialObject]))]

MeshShape.is_a = [ShapeRegion, has_file_path.exactly(1, str), has_shape_scale.max(1, float)]

MeshShapeData.is_a = [InformationObject, is_about.only(PhysicalObject)]

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

ProcessFlow.is_a = [Description, defines_process.some(ProcessType)]

PhysicsProcess.is_a = [Process, has_participant.some(PhysicalObject)]

MovingAway.is_a = [Locomotion]

MovingTo.is_a = [Navigating]

NaturalLanguage.is_a = [Language, gives_meaning_to.only(NaturalLanguageText)]

NaturalLanguageText.is_a = [Thing]
NaturalLanguageText.equivalent_to = [And([Text, is_given_meaning_by.some(NaturalLanguage)])]

Ontology.is_a = [StructuredText]
Ontology.equivalent_to = [And([StructuredText, is_given_meaning_by.some(OntologyLanguage)])]

OntologyLanguage.is_a = [KnowledgeRepresentationLanguage, gives_meaning_to.only(Ontology)]

Option.is_a = [ResourceRole]

FormalEntity.is_a = [Abstract]

Singleton.is_a = [Thing]
Singleton.equivalent_to = [And([Set, encapsulates.exactly(1, Entity)])]

Orienting.is_a = [Positioning]

Positioning.is_a = [Actuating]

Origin.is_a = [Location]

ParkingArms.is_a = [AssumingArmPose]

PhaseTransition.is_a = [Alteration]

PhysicalAccessibility.is_a = [StateType, is_state_type_of.some(Item), is_state_type_of.some(Restrictor)]

PhysicalBlockage.is_a = [StateType, is_state_type_of.some(Item), is_state_type_of.some(Restrictor)]

PhysicalExistence.is_a = [PhysicalState]

PhysicalState.is_a = [State, has_participant.some(PhysicalObject)]

PlacingTheory.is_a = [ExecutableSchematicTheory, defines.some(Destination), defines.some(Patient)]

PlanarJoint.is_a = [MovableJoint]

PluginRole.is_a = [InterfaceComponentRole, is_defined_in.some(PluginSpecification)]

Pourable.is_a = [Disposition, affords_bearer.only(PouredObject)]

PouredObject.is_a = [Patient]

Pouring.is_a = [Actuating, is_task_afforded_by.some(Pourable)]

PouringInto.is_a = [Pouring]

PouringOnto.is_a = [Pouring]

Prediction.is_a = [Prospecting]

Prospecting.is_a = [DerivingInformation]

Predilection.is_a = [SocialRelation, describes.only(Or([Preference, (And([Order, orders.only(encapsulates.only(Situation))]))]))]

SocialRelation.is_a = [Relation]

PreferenceOrder.is_a = [SocialRelation, orders.some(OrderedElement), orders.only(encapsulates.only(Description)), describes.only(Preference)]

PreferenceRegion.is_a = [SocialObjectAttribute, is_region_for.some(Preference)]

SocialObjectAttribute.is_a = [Region, is_region_for.only(SocialObject)]

PrismaticJoint.is_a = [MovableJoint, has_joint_limit.exactly(1, JointLimit)]

Progression.is_a = [Situation]
Progression.equivalent_to = [satisfies.some(ProcessFlow)]

Protector.is_a = [Restrictor]

ProximalTheory.is_a = [ImageSchemaTheory, defines.some(LocatumRole), defines.some(RelatumRole)]

PushingAway.is_a = [Pushing]

PushingDown.is_a = [Pushing]

PuttingDown.is_a = [Manipulating]

QualityTransition.is_a = [Transition]

Transition.is_a = [Situation, includes_event.some(Event), includes_object.some(Object), is_setting_for.some(Process), is_setting_for.some(And([Situation, precedes.some(And([Event, precedes.some(Situation)]))])), includes_time.min(3, TimeInterval), is_setting_for.min(2, Situation)]

Query.is_a = [Message]

QueryAnsweringTask.is_a = [Thing]
QueryAnsweringTask.equivalent_to = [And([AnsweringTask, classifies.only(is_reaction_to.only(is_classified_by.some(QueryingTask)))])]

QueryEngine.is_a = [SoftwareRole]

Reaching.is_a = [EndEffectorPositioning]

Retracting.is_a = [EndEffectorPositioning]

Reasoner.is_a = [SoftwareRole]

RecipientRole.is_a = [BeneficiaryRole]

RecordedEpisode.is_a = [Episode, includes_record.some(InformationObject)]

RedColor.is_a = [ColorRegion]

Reification.is_a = [Description, is_reification_of.exactly(1, normstr)]

RelationalDatabase.is_a = [Database]

RelationalQueryLanguage.is_a = [QueryLanguage]

RelevantPart.is_a = [Feature]

Remembering.is_a = [Thing, And([InformationRetrieval, Retrospecting])]

Retrospecting.is_a = [InformationAcquisition]

RemovedObject.is_a = [ExcludedObject]

Replanning.is_a = [Planning]

RevoluteJoint.is_a = [HingeJoint, has_joint_limit.exactly(1, JointLimit)]

PhysicalPlace.is_a = [PhysicalObject]

Surface.is_a = [PhysicalPlace]

Rubbing.is_a = [DirectedMotion]

Scene.is_a = [Situation, includes_event.some(State), satisfies.some(ImageSchemaTheory)]

Theory.is_a = [Description, has_component.some(Relation)]

SelectedObject.is_a = [Role]

Selecting.is_a = [Deciding]

SelectingItem.is_a = [GetTaskParameter, is_task_of_output_role.some(And([SelectedObject, classifies.only(PhysicalObject)]))]

SelfReflection.is_a = [MetacognitiveControlling]

Serving.is_a = [Delivering, classifies.only(And([Action, (Or([has_participant.some(PhysicalAgent), has_participant.some(PhysicalObject)]))]))]

SettingGripper.is_a = [AssumingPose]

SixDPose.is_a = [SpaceRegion, has_position_data.exactly(1, str)]

Sharpness.is_a = [Intrinsic]

Simulating.is_a = [Prospecting]

SimulationReasoner.is_a = [Reasoner]

Set.is_a = [FormalEntity]

Size.is_a = [Intrinsic]

Slicing.is_a = [Cutting]

Sluggishness.is_a = [Amateurish]

SocialState.is_a = [State, has_participant.some(Agent)]

SoftwareConfiguration.is_a = [Configuration, is_described_by.exactly(1, Software)]

SocialAgent.is_a = [Agent, SocialObject, acts_through.some(PhysicalAgent)]

SoftwareLibrary.is_a = [Software]

SourceMaterialRole.is_a = [ResourceRole]

SphereShape.is_a = [ShapeRegion, has_radius.exactly(1, float)]

Standing.is_a = [PosturalMoving]

StaticFrictionAttribute.is_a = [FrictionAttribute]

Status.is_a = [Parameter]

StatusFailure.is_a = [Region]

StimulusRole.is_a = [CausativeRole]

Stirring.is_a = [Mixing, is_task_afforded_by.some(Composing)]

Storage.is_a = [Enclosing, affords_trigger.only(StoredObject)]

StructuralDesign.is_a = [Design]

TaskInvocation.is_a = [Workflow, has_binding.only(FactualBinding), defines_task.exactly(1, Task)]

SuccessDiagnosis.is_a = [BehavioralDiagnosis]

Successfulness.is_a = [SuccessDiagnosis]

SupportState.is_a = [FunctionalControl, is_state_type_of.some(And([Supporter, classifies.only(has_disposition.some(Deposition))]))]

Supporter.is_a = [Restrictor]

SupportTheory.is_a = [ControlTheory]

SupportedObject.is_a = [ConnectedObject]

SymbolicReasoner.is_a = [Reasoner]

Tapping.is_a = [DirectedMotion]

Taxis.is_a = [BodyMovement]

TechnicalDiagnosis.is_a = [FunctionalDiagnosis, has_constituent.some(DesignedArtifact)]

Temperature.is_a = [Intrinsic, has_region.some(TemperatureRegion), has_region.only(TemperatureRegion)]

TemperatureRegion.is_a = [Region]

Tempering.is_a = [Variability, affords_setpoint.only(classifies.only(Temperature))]

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

Unsuccessfulness.is_a = [SuccessDiagnosis]

VideoData.is_a = [InformationObject]

ThreeDPosition.is_a = [SpaceRegion, has_position_data.exactly(1, str)]

Diagnosis.is_a = [Description]

Goal.is_a = [Description]

Organism.is_a = [BiologicalObject, PhysicalAgent]

PhysicalArtifact.is_a = [PhysicalObject, is_described_by.some(Plan)]

TimeInterval.is_a = [Region]

has_name_string.is_a = [DatatypeProperty, has_data_value]
has_name_string.domain = [Entity]
has_name_string.range = [str]

has_position_data.is_a = [DatatypeProperty, has_space_parameter]
has_position_data.domain = [SpaceRegion]
has_position_data.range = [str]

has_data_value.is_a = [DatatypeProperty]
has_data_value.domain = [Entity]

has_color_value.is_a = [DatatypeProperty, has_region_data_value]
has_color_value.domain = [ColorRegion]

has_region_data_value.is_a = [DatatypeProperty, has_data_value]
has_region_data_value.domain = [Region]

has_data_format.is_a = [DatatypeProperty, has_data_value]
has_data_format.domain = [InformationRealization]
has_data_format.range = [str]

has_depth.is_a = [DatatypeProperty, has_shape_parameter]
has_depth.domain = [ShapeRegion]
has_depth.range = [float]

has_shape_parameter.is_a = [DatatypeProperty, has_region_data_value]
has_shape_parameter.domain = [ShapeRegion]
has_shape_parameter.range = [Or([float, float, float, str])]

has_event_begin.is_a = [DatatypeProperty, has_event_time]
has_event_begin.domain = [Event]
has_event_begin.range = [float]

has_event_end.is_a = [DatatypeProperty, has_event_time]
has_event_end.domain = [Event]
has_event_end.range = [float]

has_event_time.is_a = [DatatypeProperty, has_data_value]
has_event_time.domain = [Event]
has_event_time.range = [float]

has_file_path.is_a = [DatatypeProperty, has_data_value]
has_file_path.domain = [Entity]
has_file_path.range = [str]

has_force_value.is_a = [DatatypeProperty, has_region_data_value]
has_force_value.domain = [ForceAttribute]
has_force_value.range = [float]

has_friction_value.is_a = [DatatypeProperty, has_region_data_value]
has_friction_value.domain = [FrictionAttribute]
has_friction_value.range = [float]

has_hsv_value.is_a = [DatatypeProperty, has_color_value]
has_hsv_value.domain = [ColorRegion]
has_hsv_value.range = [str]

has_height.is_a = [DatatypeProperty, has_shape_parameter]
has_height.domain = [ShapeRegion]
has_height.range = [float]

has_interval_begin.is_a = [DatatypeProperty, has_interval_time]
has_interval_begin.domain = [TimeInterval]
has_interval_begin.range = [float]

has_interval_end.is_a = [DatatypeProperty, has_interval_time]
has_interval_end.domain = [TimeInterval]
has_interval_end.range = [float]

has_interval_time.is_a = [DatatypeProperty, has_region_data_value]
has_interval_time.domain = [TimeInterval]
has_interval_time.range = [float]

has_joint_effort.is_a = [DatatypeProperty, has_joint_parameter]
has_joint_effort.domain = [JointState]
has_joint_effort.range = [float]

has_joint_parameter.is_a = [DatatypeProperty, has_region_data_value]
has_joint_parameter.domain = [PhysicalAttribute]
has_joint_parameter.range = [float]

has_joint_effort_limit.is_a = [DatatypeProperty, has_joint_parameter]
has_joint_effort_limit.domain = [JointLimit]
has_joint_effort_limit.range = [float]

has_joint_position.is_a = [DatatypeProperty, has_joint_parameter]
has_joint_position.domain = [JointState]
has_joint_position.range = [float]

has_joint_position_max.is_a = [DatatypeProperty, has_joint_parameter]
has_joint_position_max.domain = [JointLimit]
has_joint_position_max.range = [float]

has_joint_position_min.is_a = [DatatypeProperty, has_joint_parameter]
has_joint_position_min.domain = [JointLimit]
has_joint_position_min.range = [float]

has_joint_velocity.is_a = [DatatypeProperty, has_joint_parameter]
has_joint_velocity.domain = [JointState]
has_joint_velocity.range = [float]

has_joint_velocity_limit.is_a = [DatatypeProperty, has_joint_parameter]
has_joint_velocity_limit.domain = [JointLimit]
has_joint_velocity_limit.range = [float]

has_length.is_a = [DatatypeProperty, has_shape_parameter]
has_length.domain = [ShapeRegion]
has_length.range = [float]

has_mass_value.is_a = [DatatypeProperty, has_region_data_value]
has_mass_value.domain = [MassAttribute]
has_mass_value.range = [float]

has_persistent_identifier.is_a = [DatatypeProperty, has_data_value]
has_persistent_identifier.domain = [InformationRealization]
has_persistent_identifier.range = [str]

has_space_parameter.is_a = [DatatypeProperty, has_region_data_value]
has_space_parameter.domain = [SpaceRegion]

has_priority.is_a = [DatatypeProperty, has_data_value]
has_priority.domain = [Task]

has_rgb_value.is_a = [DatatypeProperty, has_color_value]
has_rgb_value.domain = [ColorRegion]
has_rgb_value.range = [str]

has_radius.is_a = [DatatypeProperty, has_shape_parameter]
has_radius.domain = [ShapeRegion]
has_radius.range = [float]

has_reference_frame.is_a = [DatatypeProperty, has_space_parameter]
has_reference_frame.domain = [SpaceRegion]
has_reference_frame.range = [str]

has_shape_scale.is_a = [DatatypeProperty, has_shape_parameter]
has_shape_scale.domain = [ShapeRegion]
has_shape_scale.range = [float]

has_width.is_a = [DatatypeProperty, has_shape_parameter]
has_width.domain = [ShapeRegion]
has_width.range = [float]

is_reification_of.is_a = [DatatypeProperty, has_data_value]
is_reification_of.domain = [Description]
is_reification_of.range = [normstr]

has_disposition.is_a = [ObjectProperty, has_quality]
has_disposition.domain = [Object]
has_disposition.range = [Disposition]

has_physical_component.is_a = [ObjectProperty, has_component]
has_physical_component.domain = [PhysicalObject]
has_physical_component.range = [PhysicalObject]

associated_with.is_a = [ObjectProperty, SymmetricProperty, TransitiveProperty]
associated_with.domain = [Entity]
associated_with.range = [Entity]

has_location.is_a = [ObjectProperty, associated_with]
has_location.domain = [Entity]
has_location.range = [Entity]

affects.is_a = [ObjectProperty, precedes]

precedes.is_a = [ObjectProperty, TransitiveProperty, associated_with]
precedes.domain = [Entity]
precedes.range = [Entity]

is_affected_by.is_a = [ObjectProperty, follows]

affordance_defines.is_a = [ObjectProperty, defines]
affordance_defines.domain = [Affordance]
affordance_defines.range = [Concept]

defines.is_a = [ObjectProperty, uses_concept]
defines.domain = [Description]
defines.range = [Concept]

is_defined_in_affordance.is_a = [ObjectProperty, is_defined_in]
is_defined_in_affordance.domain = [Concept]
is_defined_in_affordance.range = [Affordance]

affordance_defines_task.is_a = [ObjectProperty, affordance_defines, defines_task]
affordance_defines_task.domain = [Affordance]
affordance_defines_task.range = [Task]

defines_task.is_a = [ObjectProperty, defines]
defines_task.domain = [Description]
defines_task.range = [Task]

is_task_defined_in_affordance.is_a = [ObjectProperty, is_defined_in_affordance, is_task_defined_in]
is_task_defined_in_affordance.domain = [Task]
is_task_defined_in_affordance.range = [Affordance]

affords_bearer.is_a = [ObjectProperty, affords_concept]
affords_bearer.domain = [Disposition]
affords_bearer.range = [Role]

affords_concept.is_a = [ObjectProperty, associated_with]
affords_concept.domain = [Disposition]
affords_concept.range = [Concept]

is_bearer_afforded_by.is_a = [ObjectProperty, is_concept_afforded_by]
is_bearer_afforded_by.domain = [Role]
is_bearer_afforded_by.range = [Disposition]

is_described_by.is_a = [ObjectProperty, associated_with]
is_described_by.domain = [Entity]
is_described_by.range = [Description]

defines_bearer.is_a = [ObjectProperty, defines_role]
defines_bearer.domain = [Affordance]
defines_bearer.range = [Role]

is_concept_afforded_by.is_a = [ObjectProperty, associated_with]
is_concept_afforded_by.domain = [Concept]
is_concept_afforded_by.range = [Disposition]

affords_performer.is_a = [ObjectProperty, affords_concept]
affords_performer.domain = [Disposition]
affords_performer.range = [Role]

is_performer_afforded_by.is_a = [ObjectProperty, is_concept_afforded_by]
is_performer_afforded_by.domain = [Role]
is_performer_afforded_by.range = [Disposition]

defines_performer.is_a = [ObjectProperty, defines_role]
defines_performer.domain = [Affordance]
defines_performer.range = [Role]

affords_setpoint.is_a = [ObjectProperty, affords_concept]
affords_setpoint.domain = [Disposition]
affords_setpoint.range = [Setpoint]

is_setpoint_afforded_by.is_a = [ObjectProperty, is_concept_afforded_by]
is_setpoint_afforded_by.domain = [Setpoint]
is_setpoint_afforded_by.range = [Disposition]

defines_setpoint.is_a = [ObjectProperty, defines_parameter]
defines_setpoint.domain = [Description]
defines_setpoint.range = [Setpoint]

affords_task.is_a = [ObjectProperty, affords_concept]
affords_task.domain = [Disposition]
affords_task.range = [Task]

is_task_afforded_by.is_a = [ObjectProperty, is_concept_afforded_by]
is_task_afforded_by.domain = [Task]
is_task_afforded_by.range = [Disposition]

affords_trigger.is_a = [ObjectProperty, affords_concept]
affords_trigger.domain = [Disposition]
affords_trigger.range = [Role]

is_trigger_afforded_by.is_a = [ObjectProperty, is_concept_afforded_by]
is_trigger_afforded_by.domain = [Role]
is_trigger_afforded_by.range = [Disposition]

defines_trigger.is_a = [ObjectProperty, defines_role]
defines_trigger.domain = [Affordance]
defines_trigger.range = [Role]

after.is_a = [ObjectProperty, TransitiveProperty, follows]
after.domain = [Entity]
after.range = [Entity]

answers.is_a = [ObjectProperty, relates_to_another_role]
answers.domain = [Answer]
answers.range = [Message]

relates_to_another_role.is_a = [ObjectProperty, SymmetricProperty, associated_with]
relates_to_another_role.domain = [Role]
relates_to_another_role.range = [Role]

has_answer.is_a = [ObjectProperty, relates_to_another_role]
has_answer.domain = [Message]
has_answer.range = [Answer]

before.is_a = [ObjectProperty, TransitiveProperty, precedes]
before.domain = [Entity]
before.range = [Entity]

causes.is_a = [ObjectProperty, TransitiveProperty, affects]

is_reaction_to.is_a = [ObjectProperty, TransitiveProperty, is_affected_by]
is_reaction_to.domain = [Event]
is_reaction_to.range = [Event]

causes_transition.is_a = [ObjectProperty, is_event_included_in]
causes_transition.domain = [Event]
causes_transition.range = [Transition]

co_occurs.is_a = [ObjectProperty, SymmetricProperty, overlaps]
co_occurs.domain = [Event]
co_occurs.range = [Event]

contains.is_a = [ObjectProperty, overlaps]
contains.domain = [Entity]
contains.range = [Entity]

contains_event.is_a = [ObjectProperty, TransitiveProperty, co_occurs, contains]
contains_event.domain = [Event]
contains_event.range = [Event]

contains_object.is_a = [ObjectProperty, TransitiveProperty, contains, is_location_of]
contains_object.domain = [PhysicalObject]
contains_object.range = [PhysicalObject]

is_location_of.is_a = [ObjectProperty, associated_with]
is_location_of.domain = [Entity]
is_location_of.range = [Entity]

is_inside_of.is_a = [ObjectProperty, TransitiveProperty, is_contained_in, has_location]
is_inside_of.domain = [PhysicalObject]
is_inside_of.range = [PhysicalObject]

covers_object.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, interacts_with]
covers_object.domain = [PhysicalObject]
covers_object.range = [PhysicalObject]

interacts_with.is_a = [ObjectProperty, SymmetricProperty, associated_with]
interacts_with.domain = [PhysicalObject]
interacts_with.range = [PhysicalObject]

is_covered_by_object.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, interacts_with]
is_covered_by_object.domain = [PhysicalObject]
is_covered_by_object.range = [PhysicalObject]

defines_role.is_a = [ObjectProperty, defines]
defines_role.domain = [Description]
defines_role.range = [Role]

is_bearer_defined_in.is_a = [ObjectProperty, is_role_defined_in]
is_bearer_defined_in.domain = [Role]
is_bearer_defined_in.range = [Affordance]

defines_event_type.is_a = [ObjectProperty, defines]
defines_event_type.domain = [Description]
defines_event_type.range = [EventType]

is_event_type_defined_in.is_a = [ObjectProperty, is_defined_in]
is_event_type_defined_in.domain = [EventType]
is_event_type_defined_in.range = [Description]

defines_input.is_a = [ObjectProperty, defines_participant]
defines_input.domain = [Description]
defines_input.range = [Or([Parameter, Role])]

defines_participant.is_a = [ObjectProperty, defines]
defines_participant.domain = [Description]
defines_participant.range = [Concept]

defines_output.is_a = [ObjectProperty, defines_participant]
defines_output.domain = [Description]
defines_output.range = [Or([Parameter, Role])]

defines_parameter.is_a = [ObjectProperty, defines]
defines_parameter.domain = [Description]
defines_parameter.range = [Parameter]

is_parameter_defined_in.is_a = [ObjectProperty, is_defined_in]
is_parameter_defined_in.domain = [Parameter]
is_parameter_defined_in.range = [Description]

is_performer_defined_in.is_a = [ObjectProperty, is_role_defined_in]
is_performer_defined_in.domain = [Role]
is_performer_defined_in.range = [Description]

defines_process.is_a = [ObjectProperty, defines]
defines_process.domain = [Description]
defines_process.range = [ProcessType]

is_process_defined_in.is_a = [ObjectProperty, is_defined_in]
is_process_defined_in.domain = [ProcessType]
is_process_defined_in.range = [Description]

is_setpoint_defined_in.is_a = [ObjectProperty, is_parameter_defined_in]
is_setpoint_defined_in.domain = [Setpoint]
is_setpoint_defined_in.range = [Description]

is_trigger_defined_in.is_a = [ObjectProperty, is_role_defined_in]
is_trigger_defined_in.domain = [Role]
is_trigger_defined_in.range = [Affordance]

derived_from.is_a = [ObjectProperty, TransitiveProperty, associated_with]
derived_from.domain = [InformationObject]
derived_from.range = [InformationObject]

describes_quality.is_a = [ObjectProperty, describes]
describes_quality.domain = [Description]
describes_quality.range = [Quality]

describes.is_a = [ObjectProperty, associated_with]
describes.domain = [Description]
describes.range = [Entity]

is_quality_described_by.is_a = [ObjectProperty, is_described_by]
is_quality_described_by.domain = [Quality]
is_quality_described_by.range = [Description]

directly_causes.is_a = [ObjectProperty, causes]
directly_causes.domain = [Event]
directly_causes.range = [Event]

is_direct_reaction_to.is_a = [ObjectProperty, is_reaction_to]
is_direct_reaction_to.domain = [Event]
is_direct_reaction_to.range = [Event]

has_terminal_scene.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, has_terminal_state]
has_terminal_scene.domain = [StateTransition]
has_terminal_scene.range = [Scene]

includes_event.is_a = [ObjectProperty, is_setting_for]
includes_event.domain = [Situation]
includes_event.range = [Event]

directly_derived_from.is_a = [ObjectProperty, derived_from]

during.is_a = [ObjectProperty, TransitiveProperty, co_occurs, is_contained_in]
during.domain = [Event]
during.range = [Event]

encapsulates.is_a = [ObjectProperty, associated_with]
encapsulates.domain = [OrderedElement]
encapsulates.range = [Entity]

encodes.is_a = [ObjectProperty, SymmetricProperty, associated_with]
encodes.domain = [InformationObject]
encodes.range = [InformationObject]

executes_motion.is_a = [ObjectProperty, is_occurrence_of]
executes_motion.domain = [MotionProcess]
executes_motion.range = [Motion]

is_occurrence_of.is_a = [ObjectProperty, is_classified_by]
is_occurrence_of.domain = [Event]
is_occurrence_of.range = [EventType]

is_executed_motion_in.is_a = [ObjectProperty, is_occurring_in]
is_executed_motion_in.domain = [Motion]
is_executed_motion_in.range = [MotionProcess]

finished_by.is_a = [ObjectProperty, TransitiveProperty, co_occurs]
finished_by.domain = [Event]
finished_by.range = [Event]

finishes.is_a = [ObjectProperty, TransitiveProperty, co_occurs]
finishes.domain = [Event]
finishes.range = [Event]

first_member.is_a = [ObjectProperty, has_member]
first_member.domain = [Collection]
first_member.range = [Entity]

gives_meaning_to.is_a = [ObjectProperty, associated_with]
gives_meaning_to.domain = [System]
gives_meaning_to.range = [InformationObject]

is_given_meaning_by.is_a = [ObjectProperty, associated_with]
is_given_meaning_by.domain = [InformationObject]
is_given_meaning_by.range = [System]

has_action.is_a = [ObjectProperty, has_constituent]
has_action.domain = [Action]
has_action.range = [Action]

has_constituent.is_a = [ObjectProperty, associated_with]
has_constituent.domain = [Entity]
has_constituent.range = [Entity]

has_alteration_result.is_a = [ObjectProperty, has_region]
has_alteration_result.domain = [Action]
has_alteration_result.range = [Region]

has_region.is_a = [ObjectProperty, associated_with]
has_region.domain = [Entity]
has_region.range = [Region]

is_alteration_result_of.is_a = [ObjectProperty, is_region_for]
is_alteration_result_of.domain = [Region]
is_alteration_result_of.range = [Action]

has_binding.is_a = [ObjectProperty, has_part]
has_binding.domain = [Description]
has_binding.range = [Binding]

has_part.is_a = [ObjectProperty, TransitiveProperty, ReflexiveProperty, associated_with]
has_part.domain = [Entity]
has_part.range = [Entity]

has_binding_filler.is_a = [ObjectProperty, describes]
has_binding_filler.domain = [Binding]
has_binding_filler.range = [Entity]

has_binding_role.is_a = [ObjectProperty, has_part]
has_binding_role.domain = [Binding]
has_binding_role.range = [Or([Parameter, Role])]

has_child_link.is_a = [ObjectProperty, has_constituent]
has_child_link.domain = [Joint]
has_child_link.range = [PhysicalObject]

is_child_link_of.is_a = [ObjectProperty, is_constituent_of]
is_child_link_of.domain = [PhysicalObject]
is_child_link_of.range = [Joint]

has_color.is_a = [ObjectProperty, has_quality]
has_color.domain = [PhysicalObject]
has_color.range = [Color]

has_quality.is_a = [ObjectProperty, associated_with]
has_quality.domain = [Entity]
has_quality.range = [Quality]

is_color_of.is_a = [ObjectProperty, is_quality_of]
is_color_of.domain = [Color]
is_color_of.range = [PhysicalObject]

is_disposition_of.is_a = [ObjectProperty, is_quality_of]
is_disposition_of.domain = [Disposition]
is_disposition_of.range = [Object]

has_end_link.is_a = [ObjectProperty, has_link]
has_end_link.domain = [PhysicalObject]
has_end_link.range = [PhysicalObject]

has_link.is_a = [ObjectProperty, has_component]
has_link.domain = [PhysicalObject]
has_link.range = [PhysicalObject]

is_end_link_of.is_a = [ObjectProperty, is_link_of]
is_end_link_of.domain = [PhysicalObject]
is_end_link_of.range = [PhysicalObject]

has_execution_state.is_a = [ObjectProperty, has_region]
has_execution_state.domain = [Action]
has_execution_state.range = [ExecutionStateRegion]

has_feature.is_a = [ObjectProperty, has_constituent]
has_feature.domain = [PhysicalObject]
has_feature.range = [Feature]

is_feature_of.is_a = [ObjectProperty, is_constituent_of]
is_feature_of.domain = [Feature]
is_feature_of.range = [PhysicalObject]

has_first_step.is_a = [ObjectProperty, has_step]
has_first_step.domain = [Workflow]
has_first_step.range = [Task]

has_step.is_a = [ObjectProperty, defines_task]
has_step.domain = [Workflow]
has_step.range = [Task]

is_first_step_of.is_a = [ObjectProperty, is_step_of]
is_first_step_of.domain = [Task]
is_first_step_of.range = [Workflow]

has_friction_attribute.is_a = [ObjectProperty, has_region]
has_friction_attribute.domain = [PhysicalObject]
has_friction_attribute.range = [FrictionAttribute]

has_goal.is_a = [ObjectProperty, is_described_by]
has_goal.domain = [Entity]
has_goal.range = [Goal]

has_initial_scene.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, has_initial_state]
has_initial_scene.domain = [StateTransition]
has_initial_scene.range = [Scene]

has_initial_situation.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, has_initial_state]
has_initial_situation.domain = [SituationTransition]
has_initial_situation.range = [Situation, Not(NonmanifestedSituation)]

has_initial_state.is_a = [ObjectProperty, includes_situation]
has_initial_state.domain = [Transition]
has_initial_state.range = [Situation]

is_initial_situation_of.is_a = [ObjectProperty, is_initial_state_of]
is_initial_situation_of.domain = [Situation]
is_initial_situation_of.range = [SituationTransition]

has_input_parameter.is_a = [ObjectProperty, has_parameter]
has_input_parameter.domain = [EventType]
has_input_parameter.range = [Parameter]

has_joint_limit.is_a = [ObjectProperty, has_region]
has_joint_limit.domain = [Joint]
has_joint_limit.range = [JointLimit]

is_joint_limit_of.is_a = [ObjectProperty, is_region_for]
is_joint_limit_of.domain = [JointLimit]
is_joint_limit_of.range = [Joint]

has_joint_state.is_a = [ObjectProperty, has_region]
has_joint_state.domain = [Joint]
has_joint_state.range = [JointState]

is_joint_state_of.is_a = [ObjectProperty, is_region_for]
is_joint_state_of.domain = [JointState]
is_joint_state_of.range = [Joint]

has_component.is_a = [ObjectProperty, AsymmetricProperty, has_proper_part]
has_component.domain = [Entity]
has_component.range = [Entity]

is_link_of.is_a = [ObjectProperty, is_component_of]
is_link_of.domain = [PhysicalObject]
is_link_of.range = [PhysicalObject]

has_localization.is_a = [ObjectProperty, has_quality]
has_localization.domain = [PhysicalObject]
has_localization.range = [Localization]

is_localization_of.is_a = [ObjectProperty, is_quality_of]
is_localization_of.domain = [Localization]
is_localization_of.range = [PhysicalObject]

has_mass_attribute.is_a = [ObjectProperty, has_region]
has_mass_attribute.domain = [PhysicalObject]
has_mass_attribute.range = [MassAttribute]

is_mass_attribute_of.is_a = [ObjectProperty, is_region_for]
is_mass_attribute_of.domain = [MassAttribute]
is_mass_attribute_of.range = [PhysicalObject]

has_net_force.is_a = [ObjectProperty, has_region]
has_net_force.domain = [PhysicalObject]
has_net_force.range = [NetForce]

is_net_force_of.is_a = [ObjectProperty, is_region_for]
is_net_force_of.domain = [NetForce]
is_net_force_of.range = [PhysicalObject]

has_next_step.is_a = [ObjectProperty, directly_precedes]
has_next_step.domain = [Task]
has_next_step.range = [Task]

directly_precedes.is_a = [ObjectProperty, precedes]
directly_precedes.domain = [Entity]
directly_precedes.range = [Entity]

has_previous_step.is_a = [ObjectProperty, directly_follows]
has_previous_step.domain = [Task]
has_previous_step.range = [Task]

has_output_parameter.is_a = [ObjectProperty, has_parameter]
has_output_parameter.domain = [EventType]
has_output_parameter.range = [Parameter]

has_parent_link.is_a = [ObjectProperty, has_constituent]
has_parent_link.domain = [Joint]
has_parent_link.range = [PhysicalObject]

is_parent_link_of.is_a = [ObjectProperty, is_constituent_of]
is_parent_link_of.domain = [PhysicalObject]
is_parent_link_of.range = [Joint]

has_phase.is_a = [ObjectProperty, has_constituent]
has_phase.domain = [Or([Action, Process])]
has_phase.range = [Or([State, Process])]

has_predecessor.is_a = [ObjectProperty, has_part]
has_predecessor.domain = [Succedence]
has_predecessor.range = [Workflow]

has_preference.is_a = [ObjectProperty, has_quality]
has_preference.domain = [Agent]
has_preference.range = [Preference]

is_preference_of.is_a = [ObjectProperty, is_quality_of]
is_preference_of.domain = [Preference]
is_preference_of.range = [Agent]

directly_follows.is_a = [ObjectProperty, follows]
directly_follows.domain = [Entity]
directly_follows.range = [Entity]

has_process_type.is_a = [ObjectProperty, is_related_to_concept]
has_process_type.domain = [Role]
has_process_type.range = [ProcessType]

is_related_to_concept.is_a = [ObjectProperty, SymmetricProperty, associated_with]
is_related_to_concept.domain = [Concept]
is_related_to_concept.range = [Concept]

is_process_type_of.is_a = [ObjectProperty, is_related_to_concept]
is_process_type_of.domain = [ProcessType]
is_process_type_of.range = [Role]

has_quale.is_a = [ObjectProperty, has_region]
has_quale.domain = [Quality]
has_quale.range = [Region]

is_quale_of.is_a = [ObjectProperty, is_region_for]
is_quale_of.domain = [Region]
is_quale_of.range = [Quality]

has_root_link.is_a = [ObjectProperty, has_link]
has_root_link.domain = [PhysicalObject]
has_root_link.range = [PhysicalObject]

is_root_link_of.is_a = [ObjectProperty, is_link_of]
is_root_link_of.domain = [PhysicalObject]
is_root_link_of.range = [PhysicalObject]

has_shape.is_a = [ObjectProperty, has_quality]
has_shape.domain = [PhysicalObject]
has_shape.range = [Shape]

is_shape_of.is_a = [ObjectProperty, is_quality_of]
is_shape_of.domain = [Shape]
is_shape_of.range = [PhysicalObject]

has_shape_region.is_a = [ObjectProperty, has_region]
has_shape_region.domain = [Or([Shape, PhysicalObject])]
has_shape_region.range = [ShapeRegion]

is_shape_region_of.is_a = [ObjectProperty, is_region_for]
is_shape_region_of.domain = [ShapeRegion]
is_shape_region_of.range = [Or([Shape, PhysicalObject])]

has_software_agent.is_a = [ObjectProperty, involves_agent]
has_software_agent.domain = [Event]
has_software_agent.range = [SoftwareInstance]

has_space_region.is_a = [ObjectProperty, has_region]
has_space_region.domain = [Or([Localization, ShapeRegion, PhysicalObject])]
has_space_region.range = [SpaceRegion]

is_space_region_for.is_a = [ObjectProperty, is_region_for]
is_space_region_for.domain = [SpaceRegion]
is_space_region_for.range = [Or([Localization, PhysicalObject])]

has_state_type.is_a = [ObjectProperty, is_related_to_concept]
has_state_type.domain = [Role]
has_state_type.range = [StateType]

is_state_type_of.is_a = [ObjectProperty, is_related_to_concept]
is_state_type_of.domain = [StateType]
is_state_type_of.range = [Role]

has_status.is_a = [ObjectProperty, has_quality]

is_step_of.is_a = [ObjectProperty, is_task_defined_in]
is_step_of.domain = [Task]
is_step_of.range = [Workflow]

has_succedence.is_a = [ObjectProperty, has_part]
has_succedence.domain = [Workflow]
has_succedence.range = [Succedence]

has_successor.is_a = [ObjectProperty, has_part]
has_successor.domain = [Succedence]
has_successor.range = [Workflow]

has_task.is_a = [ObjectProperty, has_part]
has_task.domain = [Or([Relation, Workflow])]
has_task.range = [Task]

has_terminal_situation.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, has_terminal_state]
has_terminal_situation.domain = [SituationTransition]
has_terminal_situation.range = [Situation, Not(NonmanifestedSituation)]

has_terminal_state.is_a = [ObjectProperty, includes_situation]
has_terminal_state.domain = [Transition]
has_terminal_state.range = [Situation]

is_terminal_situation_of.is_a = [ObjectProperty, is_terminal_state_of]
is_terminal_situation_of.domain = [Situation]
is_terminal_situation_of.range = [SituationTransition]

includes_concept.is_a = [ObjectProperty, includes_object]
includes_concept.domain = [Situation]
includes_concept.range = [Concept]

includes_record.is_a = [ObjectProperty, is_reference_of]
includes_record.domain = [Event]
includes_record.range = [InformationObject]

includes_situation.is_a = [ObjectProperty, is_setting_for]
includes_situation.domain = [Situation]
includes_situation.range = [Situation]

involves_artifact.is_a = [ObjectProperty, has_participant]
involves_artifact.domain = [Event]
involves_artifact.range = [PhysicalArtifact]

involves_effector.is_a = [ObjectProperty, has_participant]
involves_effector.domain = [Event]
involves_effector.range = [PhysicalEffector]

has_participant.is_a = [ObjectProperty, associated_with]
has_participant.domain = [Event]
has_participant.range = [Object]

is_effector_involved_in.is_a = [ObjectProperty, is_participant_in]
is_effector_involved_in.domain = [PhysicalEffector]
is_effector_involved_in.range = [Event]

involves_place.is_a = [ObjectProperty, has_participant]
involves_place.domain = [Event]
involves_place.range = [PhysicalPlace]

follows.is_a = [ObjectProperty, TransitiveProperty, associated_with]
follows.domain = [Entity]
follows.range = [Entity]

is_region_for.is_a = [ObjectProperty, associated_with]
is_region_for.domain = [Region]
is_region_for.range = [Entity]

is_answered_by.is_a = [ObjectProperty, involves_agent]
is_answered_by.domain = [Event]
is_answered_by.range = [Agent]

involves_agent.is_a = [ObjectProperty, has_participant]
involves_agent.domain = [Event]
involves_agent.range = [Agent]

is_artifact_involved_in.is_a = [ObjectProperty, is_participant_in]
is_artifact_involved_in.domain = [PhysicalArtifact]
is_artifact_involved_in.range = [Event]

is_asked_by.is_a = [ObjectProperty, involves_agent]
is_asked_by.domain = [QueryingTask]
is_asked_by.range = [Agent]

is_role_defined_in.is_a = [ObjectProperty, is_defined_in]
is_role_defined_in.domain = [Role]
is_role_defined_in.range = [Description]

is_caused_by_event.is_a = [ObjectProperty, includes_event]
is_caused_by_event.domain = [Transition]
is_caused_by_event.range = [Event]

is_constituent_of.is_a = [ObjectProperty, associated_with]
is_constituent_of.domain = [Entity]
is_constituent_of.range = [Entity]

is_quality_of.is_a = [ObjectProperty, associated_with]
is_quality_of.domain = [Quality]
is_quality_of.range = [Entity]

is_concept_included_in.is_a = [ObjectProperty, is_object_included_in]
is_concept_included_in.domain = [Concept]
is_concept_included_in.range = [Situation]

is_contained_in.is_a = [ObjectProperty, overlaps]
is_contained_in.domain = [Entity]
is_contained_in.range = [Entity]

is_created_output_of.is_a = [ObjectProperty, is_output_role_of]
is_created_output_of.domain = [Role]
is_created_output_of.range = [Task]

is_output_role_of.is_a = [ObjectProperty, has_task]
is_output_role_of.domain = [Role]
is_output_role_of.range = [Task]

is_task_of_created_role.is_a = [ObjectProperty, is_task_of_output_role]
is_task_of_created_role.domain = [Task]
is_task_of_created_role.range = [Role]

is_defined_in.is_a = [ObjectProperty, is_concept_used_in]
is_defined_in.domain = [Concept]
is_defined_in.range = [Description]

is_deposit_of.is_a = [ObjectProperty, is_location_of]
is_deposit_of.domain = [PhysicalObject]
is_deposit_of.range = [PhysicalObject]

is_ontop_of.is_a = [ObjectProperty, has_location]
is_ontop_of.domain = [PhysicalObject]
is_ontop_of.range = [PhysicalObject]

is_design_for.is_a = [ObjectProperty, describes]
is_design_for.domain = [Design]
is_design_for.range = [Object]

is_designed_by.is_a = [ObjectProperty, is_described_by]
is_designed_by.domain = [Object]
is_designed_by.range = [Design]

is_direct_source_for.is_a = [ObjectProperty, is_source_for]

is_realized_by.is_a = [ObjectProperty, associated_with]
is_realized_by.domain = [InformationObject]
is_realized_by.range = [InformationRealization]

has_role.is_a = [ObjectProperty, is_classified_by]
has_role.domain = [Object]
has_role.range = [Role]

is_input_role_of.is_a = [ObjectProperty, has_task]
is_input_role_of.domain = [Role]
is_input_role_of.range = [Task]

is_role_of.is_a = [ObjectProperty, classifies]
is_role_of.domain = [Role]
is_role_of.range = [Object]

realizes.is_a = [ObjectProperty, associated_with]
realizes.domain = [InformationRealization]
realizes.range = [InformationObject]

is_participant_in.is_a = [ObjectProperty, associated_with]
is_participant_in.domain = [Object]
is_participant_in.range = [Event]

is_occurring_in.is_a = [ObjectProperty, classifies]
is_occurring_in.domain = [EventType]
is_occurring_in.range = [Event]

is_executor_defined_in.is_a = [ObjectProperty]

is_initial_scene_of.is_a = [ObjectProperty, is_initial_state_of]
is_initial_scene_of.domain = [Scene]
is_initial_scene_of.range = [StateTransition]

is_initial_state_of.is_a = [ObjectProperty, is_situation_included_in]
is_initial_state_of.domain = [Situation]
is_initial_state_of.range = [Transition]

is_input_parameter_for.is_a = [ObjectProperty, is_parameter_for]
is_input_parameter_for.domain = [Parameter]
is_input_parameter_for.range = [EventType]

is_component_of.is_a = [ObjectProperty, AsymmetricProperty, is_propert_part_of]
is_component_of.domain = [Entity]
is_component_of.range = [Entity]

is_linked_to.is_a = [ObjectProperty, SymmetricProperty, has_location]
is_linked_to.domain = [PhysicalObject]
is_linked_to.range = [PhysicalObject]

is_motion_description_for.is_a = [ObjectProperty, describes]
is_motion_description_for.domain = [MotionDescription]
is_motion_description_for.range = [Motion]

is_moved_by_agent.is_a = [ObjectProperty, interacts_with]
is_moved_by_agent.domain = [PhysicalObject]
is_moved_by_agent.range = [Agent]

moves_object.is_a = [ObjectProperty, interacts_with]
moves_object.domain = [Agent]
moves_object.range = [PhysicalObject]

is_ordered_by.is_a = [ObjectProperty, associated_with]
is_ordered_by.domain = [OrderedElement]
is_ordered_by.range = [Order]

orders.is_a = [ObjectProperty, associated_with]
orders.domain = [Order]
orders.range = [OrderedElement]

is_output_parameter_for.is_a = [ObjectProperty, is_parameter_for]
is_output_parameter_for.domain = [Parameter]
is_output_parameter_for.range = [EventType]

is_performed_by.is_a = [ObjectProperty, involves_agent]
is_performed_by.domain = [Action]
is_performed_by.range = [Agent]

is_physically_contained_in.is_a = [ObjectProperty, is_location_of]
is_physically_contained_in.domain = [Entity]
is_physically_contained_in.range = [Entity]

is_place_involved_in.is_a = [ObjectProperty, is_participant_in]
is_place_involved_in.domain = [PhysicalPlace]
is_place_involved_in.range = [Event]

is_plan_for.is_a = [ObjectProperty, describes]
is_plan_for.domain = [Plan]
is_plan_for.range = [Task]

is_record_included_by.is_a = [ObjectProperty, is_about]
is_record_included_by.domain = [InformationObject]
is_record_included_by.range = [Event]

is_replaced_by.is_a = [ObjectProperty, before]
is_replaced_by.domain = [State]
is_replaced_by.range = [State]

replaces.is_a = [ObjectProperty, after]
replaces.domain = [State]
replaces.range = [State]

is_event_included_in.is_a = [ObjectProperty, has_setting]
is_event_included_in.domain = [Event]
is_event_included_in.range = [Situation]

is_situation_included_in.is_a = [ObjectProperty, has_setting]
is_situation_included_in.domain = [Situation]
is_situation_included_in.range = [Situation]

is_source_for.is_a = [ObjectProperty, TransitiveProperty, associated_with]
is_source_for.domain = [InformationObject]
is_source_for.range = [InformationObject]

is_task_defined_in.is_a = [ObjectProperty, is_defined_in]
is_task_defined_in.domain = [Task]
is_task_defined_in.range = [Description]

is_supported_by.is_a = [ObjectProperty, interacts_with, has_common_boundary]
is_supported_by.domain = [Entity]
is_supported_by.range = [Entity]

has_common_boundary.is_a = [ObjectProperty, SymmetricProperty, associated_with]
has_common_boundary.domain = [Entity]
has_common_boundary.range = [Entity]

supports.is_a = [ObjectProperty, interacts_with, has_common_boundary]
supports.domain = [Entity]
supports.range = [Entity]

is_task_of_output_role.is_a = [ObjectProperty, is_task_of]
is_task_of_output_role.domain = [Task]
is_task_of_output_role.range = [Role]

is_task_of_input_role.is_a = [ObjectProperty, is_task_of]
is_task_of_input_role.domain = [Task]
is_task_of_input_role.range = [Role]

is_terminal_scene_of.is_a = [ObjectProperty, is_terminal_state_of]
is_terminal_scene_of.domain = [Scene]
is_terminal_scene_of.range = [StateTransition]

is_terminal_state_of.is_a = [ObjectProperty, is_situation_included_in]
is_terminal_state_of.domain = [Situation]
is_terminal_state_of.range = [Transition]

is_terminated_by.is_a = [ObjectProperty, associated_with]
is_terminated_by.domain = [Event]
is_terminated_by.range = [Event]

terminates.is_a = [ObjectProperty, associated_with]
terminates.domain = [Event]
terminates.range = [Event]

meets.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, directly_precedes]
meets.domain = [Entity]
meets.range = [Entity]

met_by.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, directly_follows]
met_by.domain = [Entity]
met_by.range = [Entity]

overlapped_by.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, overlaps]
overlapped_by.domain = [Entity]
overlapped_by.range = [Entity]

overlapped_on.is_a = [ObjectProperty, AsymmetricProperty, IrreflexiveProperty, overlaps]
overlapped_on.domain = [Entity]
overlapped_on.range = [Entity]

simultaneous.is_a = [ObjectProperty, SymmetricProperty, TransitiveProperty, co_occurs]
simultaneous.domain = [Event]
simultaneous.range = [Event]

started_by.is_a = [ObjectProperty, TransitiveProperty, co_occurs]
started_by.domain = [Event]
started_by.range = [Event]

starts.is_a = [ObjectProperty, TransitiveProperty, co_occurs]
starts.domain = [Event]
starts.range = [Event]

transitions_back.is_a = [ObjectProperty, transitions_from, transitions_to]
transitions_back.domain = [Transient]
transitions_back.range = [Object]

transitions_from.is_a = [ObjectProperty, has_common_boundary]
transitions_from.domain = [Transient]
transitions_from.range = [Object]

transitions_to.is_a = [ObjectProperty, has_common_boundary]
transitions_to.domain = [Transient]
transitions_to.range = [Object]

has_expected_terminal_situation.is_a = [ObjectProperty, has_terminal_situation, has_postcondition]
has_expected_terminal_situation.domain = [SituationTransition]
has_expected_terminal_situation.range = [Situation]

has_postcondition.is_a = [ObjectProperty, directly_precedes]
has_postcondition.domain = [Or([Event, Situation])]
has_postcondition.range = [Or([Event, Situation])]

has_required_initial_situation.is_a = [ObjectProperty, has_initial_situation, has_precondition]
has_required_initial_situation.domain = [SituationTransition]
has_required_initial_situation.range = [Situation]

has_precondition.is_a = [ObjectProperty, directly_follows]
has_precondition.domain = [Or([Event, Situation])]
has_precondition.range = [Or([Event, Situation])]

manifests_in.is_a = [ObjectProperty, includes_event]

prevented_by.is_a = [ObjectProperty, has_setting]
prevented_by.domain = [Situation]
prevented_by.range = [Situation]

has_setting.is_a = [ObjectProperty, associated_with]
has_setting.domain = [Entity]
has_setting.range = [Situation]

prevents.is_a = [ObjectProperty, is_setting_for]
prevents.domain = [Situation]
prevents.range = [Situation]

is_setting_for.is_a = [ObjectProperty, associated_with]
is_setting_for.domain = [Situation]
is_setting_for.range = [Entity]

acts_for.is_a = [ObjectProperty, associated_with]
acts_for.domain = [Agent]
acts_for.range = [SocialAgent]

classifies.is_a = [ObjectProperty, associated_with]
classifies.domain = [Concept]
classifies.range = [Entity]

executes_task.is_a = [ObjectProperty, is_classified_by]
executes_task.domain = [Action]
executes_task.range = [Task]

expresses.is_a = [ObjectProperty, associated_with]
expresses.domain = [InformationObject]
expresses.range = [SocialObject]

is_about.is_a = [ObjectProperty, associated_with]
is_about.domain = [InformationObject]
is_about.range = [Entity]

has_member.is_a = [ObjectProperty, associated_with]
has_member.domain = [Collection]
has_member.range = [Entity]

has_parameter.is_a = [ObjectProperty, is_related_to_concept]
has_parameter.domain = [Concept]
has_parameter.range = [Parameter]

has_task.is_a = [ObjectProperty, is_related_to_concept]
has_task.domain = [Role]
has_task.range = [Task]

includes_object.is_a = [ObjectProperty, is_setting_for]
includes_object.domain = [Situation]
includes_object.range = [Object]

is_classified_by.is_a = [ObjectProperty, associated_with]
is_classified_by.domain = [Entity]
is_classified_by.range = [Concept]

is_executed_in.is_a = [ObjectProperty, classifies]
is_executed_in.domain = [Task]
is_executed_in.range = [Action]

is_expressed_by.is_a = [ObjectProperty, associated_with]
is_expressed_by.domain = [SocialObject]
is_expressed_by.range = [InformationObject]

is_reference_of.is_a = [ObjectProperty, associated_with]
is_reference_of.domain = [Entity]
is_reference_of.range = [InformationObject]

is_object_included_in.is_a = [ObjectProperty, has_setting]
is_object_included_in.domain = [Object]
is_object_included_in.range = [Situation]

is_parameter_for.is_a = [ObjectProperty, is_related_to_concept]
is_parameter_for.domain = [Parameter]
is_parameter_for.range = [Concept]

is_part_of.is_a = [ObjectProperty, TransitiveProperty, ReflexiveProperty, associated_with]
is_part_of.domain = [Entity]
is_part_of.range = [Entity]

is_task_of.is_a = [ObjectProperty, is_related_to_concept]
is_task_of.domain = [Task]
is_task_of.range = [Role]

overlaps.is_a = [ObjectProperty, SymmetricProperty, associated_with]
overlaps.domain = [Entity]
overlaps.range = [Entity]

satisfies.is_a = [ObjectProperty, associated_with]
satisfies.domain = [Situation]
satisfies.range = [Description]




