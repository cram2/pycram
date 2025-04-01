import inspect
import logging
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from inspect import isbuiltin

from ormatic.ormatic import ORMatic, logger
from sqlacodegen.generators import TablesGenerator
from sqlalchemy import create_engine, DateTime
from sqlalchemy.orm import registry, Session
from typing_extensions import Dict, Optional, Union, Type, Sequence

from pycram import World
from pycram.datastructures.dataclasses import FrozenObject
from pycram.datastructures.enums import Arms, DetectionTechnique, DetectionState, Grasp, GripperState, TorsoState
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import Pose
from pycram.description import ObjectDescription
from pycram.designator import ActionDescription
from pycram.designators.action_designator import MoveTorsoAction, SetGripperAction, \
    ReleaseAction, GripAction, ParkArmsAction,  PickUpAction, PlaceAction, NavigateAction, TransportAction, LookAtAction, \
    DetectAction, OpenAction, CloseAction, GraspingAction
from pycram.designators.object_designator import ObjectPart
from pycram.process_module import simulated_robot
from pycram.ros import Time
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Location, Agent


# ----------------------------------------------------------------------------------------------------------------------
#            All Designators to be mapped. Specify the columns that are supposed to be tracked in the database.
#            One attribute equals one column.
# ----------------------------------------------------------------------------------------------------------------------
@dataclass
class AgentSQL:
    pass


@dataclass
class PositionSQL:
    x: float
    y: float
    z: float


@dataclass
class QuaternionSQL:
    x: float
    y: float
    z: float
    w: float


@dataclass
class PoseSQL:
    position: PositionSQL
    orientation: QuaternionSQL
    frame: str
    header: datetime


@dataclass
class ObjectSQL:
    name: str
    pose: Optional[PoseSQL] = None


@dataclass
class ActionSQL:
    robot_position: PoseSQL
    robot_torso_height: float
    # robot_type: Agent


@dataclass
class MoveTorsoActionSQL(ActionSQL):
    torso_state: TorsoState


@dataclass
class SetGripperActionSQL(ActionSQL):
    gripper: Arms
    motion: GripperState


@dataclass
class ReleaseActionSQL(ActionSQL):
    gripper: str
    object_designator: ObjectSQL


@dataclass
class GripActionSQL(ActionSQL):
    gripper: str
    effort: float
    object_designator: ObjectSQL


@dataclass
class ParkArmsActionSQL(ActionSQL):
    arm: Arms


# @dataclass
# class ReachToPickUpActionSQL(ActionSQL):
#     arm: Arms
#     prepose_distance: float
#     grasp_description: GraspDescription
#     object_designator: ObjectDesignatorDescription.Object
#     orm_object_at_execution: ObjectDesignatorDescription.Object


@dataclass
class PickUpActionSQL(ActionSQL):
    arm: Arms
    prepose_distance: float
    grasp_description: GraspDescription
    object_designator: ObjectSQL
    orm_object_at_execution: Optional[FrozenObject]


@dataclass
class PlaceActionSQL(ActionSQL):
    arm: Arms
    target_location: Pose
    object_designator: ObjectSQL


@dataclass
class NavigateActionSQL(ActionSQL):
    target_location: Pose
    keep_joint_states: bool


@dataclass
class TransportActionSQL(ActionSQL):
    arm: Arms
    target_location: Pose
    pickup_prepose_distance: float
    object_designator: ObjectSQL


@dataclass
class LookAtActionSQL(ActionSQL):
    target_location: Pose


@dataclass
class DetectActionSQL(ActionSQL):
    technique: DetectionTechnique
    state: DetectionState
    object_designator_description: Optional[ObjectSQL]
    region: Optional[Location]
    orm_object_at_execution: Optional[FrozenObject]


@dataclass
class OpenActionSQL(ActionSQL):
    arm: Arms
    grasping_prepose_distance: float
    object_designator: ObjectDescription.Link


@dataclass
class CloseActionSQL(ActionSQL):
    arm: Arms
    grasping_prepose_distance: float
    object_designator: ObjectDescription.Link


@dataclass
class GraspingActionSQL(ActionSQL):
    arm: Arms
    prepose_distance: float
    # object_designator: Union[Object, ObjectDescription.Link]


@dataclass
class FaceAtActionSQL(ActionSQL):
    pose: Pose
    keep_joint_states: bool


@dataclass
class MoveAndPickUpActionSQL(ActionSQL):
    standing_position: Pose
    object_designator: ObjectSQL
    arm: Arms
    grasp: Grasp
    keep_joint_states: bool
    pickup_prepose_distance: float


# @dataclass
# class MoveAndPlaceActionSQL(ActionSQL):
#     standing_position: Pose
#     object_designator: ObjectDesignatorDescription.Object
#     target_location: Pose
#     arm: Arms
#     keep_joint_states: bool
#
#
# @dataclass
# class PouringActionSQL(ActionSQL):
#     object_: ObjectDesignatorDescription
#     tool: ObjectDesignatorDescription
#     arm: Arms
#     technique: Optional[str]
#     angle: Optional[float]


"""
Dictionary of mapped actions to their corresponding SQL classes.
Required for automatic table creation.
Make sure to add all classes that are used in the ORM to this dictionary.
"""
mapped_action_designators = {
    Pose: PoseSQL,
    Pose.position: PositionSQL,
    Pose.orientation: QuaternionSQL,

    Object: ObjectSQL,

    ActionDescription: ActionSQL,
    MoveTorsoAction: MoveTorsoActionSQL,
    SetGripperAction: SetGripperActionSQL,
    ReleaseAction: ReleaseActionSQL,
    GripAction: GripActionSQL,
    ParkArmsAction: ParkArmsActionSQL,
    PickUpAction: PickUpActionSQL,
    PlaceAction: PlaceActionSQL,
    NavigateAction: NavigateActionSQL,
    TransportAction: TransportActionSQL,
    LookAtAction: LookAtActionSQL,
    DetectAction: DetectActionSQL,
    OpenAction: OpenActionSQL,
    CloseAction: CloseActionSQL,
    GraspingAction: GraspingActionSQL,
    # FaceAtAction.Action: FaceAtActionSQL,
    # MoveAndPickUpAction.Action: MoveAndPickUpActionSQL
}

mapped_designators = {
    Pose: PoseSQL,
    Pose.position: PositionSQL,
    Pose.orientation: QuaternionSQL,
    Object: ObjectSQL,
}

def setup_logging_and_create_tables():
    """
    Set up logging and create tables for ORMatic.
    """
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))

    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    mapper_registry = registry()
    engine = create_engine('sqlite:///:memory:')
    session = Session(engine)

    ormatic = ORMatic(list(mapped_action_designators.values()) + list(mapped_designators.values()), mapper_registry)
    ormatic.make_all_tables()
    mapper_registry.metadata.create_all(session.bind)

    generator = TablesGenerator(mapper_registry.metadata, session.bind, [])

    path = os.path.abspath(os.path.join(os.getcwd(), '../src/pycram/orm/'))
    with open(os.path.join(path, 'ormatic_designator_interface.py'), 'w') as f:
        ormatic.to_python_file(generator, f)

    return session


# session = setup_logging_and_create_tables()


def to_sql(cls, session: Session, is_action_class: bool = True):
    """
    Convert an action class to its corresponding ORM class and add its data to the database.
    """
    print("Converting action class to SQL")
    if not is_action_class:
        orm_class = mapped_designators[type(cls)]
    else:
        orm_class = mapped_action_designators[type(cls)]
    orm_class_variables = inspect.getfullargspec(orm_class.__init__).args
    print("class", cls)
    print("orm class", orm_class)
    print(orm_class_variables)

    # class_variables = cls.__dict__
    # if class_variables == {}:
    #     class_variables = inspect.getfullargspec(cls).args

    class_variables = {name: value for name, value in inspect.getmembers(cls) if not callable(value) and not name.startswith("__")}    # class_variables = vars(cls)
    print(class_variables)
    for key, value in class_variables.items():
        print(type(value), key)
        if isinstance(value, Pose):
            to_sql(value, session, is_action_class=False)
        # if key == "position":
        #     class_variables[key] = PositionSQL(**value.position_as_list())
        # if key == "orientation":
        #     class_variables[key] = QuaternionSQL(**dict(zip(["x", "y", "z", "w"], value.orientation_as_list())))

    orm_instance = orm_class(**{key: value for key, value in class_variables.items() if key in orm_class_variables})
    # orm_instance = orm_class(**[getattr(cls, element) for element in class_variables if element in orm_class_variables])
    session.add(orm_instance)
    session.commit()

def add_callbacks_for_mapped_classes(session: Session):
    """
    Add callbacks for mapped classes to the ORMatic instance.
    This is used to automatically add data to the database when an action is performed.
    """
    ActionDescription.post_perform(lambda self: to_sql(self, session))
    # to_sql(Pose, session, is_action_class=False)



