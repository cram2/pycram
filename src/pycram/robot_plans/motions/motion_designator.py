from dataclasses import dataclass
from pycrap.ontologies import PhysicalObject, Location
from pycram.designators.object_designator import ObjectDesignatorDescription, ObjectPart
from pycram.datastructures.enums import MovementType, WaypointsMovementType
from pycram.failure_handling import try_motion
from pycram.failures import PerceptionObjectNotFound, ToolPoseNotReachedError
from pycram.object_descriptors.urdf import LinkDescription, ObjectDescription
from pycram.plan import with_plan
from pycram.process_module import ProcessModuleManager
from pycram.datastructures.enums import ObjectType, Arms, GripperState, ExecutionType, DetectionTechnique, DetectionState

from typing_extensions import Dict, Optional, Type, List
from pycram.datastructures.pose import PoseStamped, Vector3Stamped
from pycram.designator import BaseMotion
from pycram.world_concepts.world_object import Object
from pycram.external_interfaces.robokudo import robokudo_found


















