from .dependencies import *
from .classes import *
from .object_properties import *
from .data_properties import *


ExecutionState_Active = ExecutionStateRegion(namespace = ontology)
ExecutionState_Cancelled = ExecutionStateRegion(namespace = ontology)
ExecutionState_Failed = ExecutionStateRegion(namespace = ontology)
ExecutionState_Paused = ExecutionStateRegion(namespace = ontology)
ExecutionState_Pending = ExecutionStateRegion(namespace = ontology)
ExecutionState_Succeeded = ExecutionStateRegion(namespace = ontology)
FailWFNoContinuation = StatusFailure(namespace = ontology)
FailWFNondeterministicContinuation = StatusFailure(namespace = ontology)
FailWFUninterpretableTask = StatusFailure(namespace = ontology)
RDFType = Reification(namespace = ontology)

ExecutionState_Active.comment = ['The execution state of an ongoing activity.']

ExecutionState_Cancelled.comment = ['The execution state of a cancelled activity.']

ExecutionState_Failed.comment = ['The execution state of a failed activity.']

ExecutionState_Paused.comment = ['The execution state of a paused activity.']

ExecutionState_Pending.comment = ['The execution state of a pending activity.']

ExecutionState_Succeeded.comment = ['The execution state of a succeeded activity.']

FailWFNoContinuation.comment = ['A particular kind of failure: the workflow fails because there is no way to continue it, and the normal exit has not been reached.']

FailWFNondeterministicContinuation.comment = ['A particular kind of failure: it is not clear how to continue a workflow because several possibilities exist.']

FailWFUninterpretableTask.comment = ['A particular kind of failure: a task is not recognized and not associated to anything that could execute it, nor to a workflow that could detail its structure into simpler structure.']

# RDFType.is_reification_of = ['http://www.w3.org/2001/XMLSchema#type']

