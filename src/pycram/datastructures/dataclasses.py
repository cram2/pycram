from __future__ import annotations

import math

from dataclasses import dataclass, fields, field
from enum import Enum

import numpy as np
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.world import World

from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.world_modification import WorldModelModificationBlock
from typing_extensions import List, Optional, Callable, Dict, Any, Union, Sequence, \
    deprecated, Type, TYPE_CHECKING

from .enums import JointType, VirtualMobileBaseJointName, Grasp, ApproachDirection, VerticalAlignment
from .pose import PoseStamped, Point

if TYPE_CHECKING:
    from ..plan import Plan


@dataclass
class Context:
    """
    A dataclass for storing the context of a plan
    """
    world: World
    """
    The world in which the plan is executed
    """

    robot: AbstractRobot
    """
    The semantic robot annotation which should execute the plan
    """

    super_plan: Optional[Plan] = field(default=None)
    """
    The plan of which this plan/designator is a part of
    """

    ros_node: Optional[Any] = field(default=None)
    """
    A ROS node that should be used for communication in this plan
    """

    @classmethod
    def from_world(cls, world: World, super_plan: Optional[Plan] = None):
        """
        Create a context from a world by getting the first robot in the world. There is no super plan in this case.

        :param world: The world for which to create the context
        :param super_plan: An optional super plan
        :return: A context with the first robot in the world and no super plan
        """
        return cls(world=world, robot=world.get_semantic_annotations_by_type(AbstractRobot)[0], super_plan=super_plan)

    @classmethod
    def from_plan(cls, plan: Plan):
        """
        Create a context from a plan by getting the context information from the plan and setting the super plan to
        the given plan.

        :param plan: Plan from which to create the context
        :return: A new context with the world and robot from the plan and the super plan set to the given plan
        """
        return cls(world=plan.world, robot=plan.robot, super_plan=plan)


@dataclass
class ExecutionData:
    """
    A dataclass for storing the information of an execution that is used for creating a robot description for that
    execution. An execution is a Robot with a virtual mobile base that can be used to move the robot in the environment.
    """
    execution_start_pose: PoseStamped
    """
    Start of the robot at the start of execution of an action designator
    """

    execution_start_world_state: np.ndarray
    """
    The world state at the start of execution of an action designator
    """

    execution_end_pose: Optional[PoseStamped]  = None
    """
    The pose of the robot at the end of executing an action designator
    """

    execution_end_world_state: Optional[np.ndarray] = None
    """
    The world state at the end of executing an action designator
    """

    added_world_modifications: List[WorldModelModificationBlock] = field(default_factory=list)
    """
    A list of World modification blocks that were added during the execution of the action designator
    """

    manipulated_body_pose_start: Optional[PoseStamped] = None
    """
    Start pose of the manipulated Body if there was one
    """

    manipulated_body_pose_end: Optional[PoseStamped] = None
    """
    End pose of the manipulated Body if there was one
    """

    manipulated_body: Optional[Body] = None
    """
    Reference to the manipulated body 
    """

    manipulated_body_name: Optional[str] = None
    """
    Name of the manipulated body
    """


@dataclass
class ManipulatorData:
    """
    A dataclass for storing the information of a manipulator that is used for creating a robot description for that
    manipulator. A manipulator is an Arm with an end-effector that can be used to interact with the environment.
    """
    name: str
    """
    Name of the Manipulator.
    """
    base_link: str
    """
    Manipulator's base link.
    """
    arm_end_link: str
    """
    Manipulator's arm end link.
    """
    joint_names: List[str]
    """
    List of joint names.
    """
    home_joint_values: List[float]
    """
    List of joint values for the home position. (default position)
    """
    gripper_name: str
    """
    Name of the gripper at the end of the arm.
    """
    gripper_tool_frame: str
    """
    Name of the frame of the gripper tool.
    """
    gripper_joint_names: List[str]
    """
    List of gripper joint names.
    """
    closed_joint_values: List[float]
    """
    List of joint values for the gripper in the closed position.
    """
    open_joint_values: List[float]
    """
    List of joint values for the gripper in the open position.
    """
    opening_distance: float
    """
    The opening distance of the gripper.
    """
    fingers_link_names: Optional[List[str]] = None
    """
    List of link names for the fingers of the gripper.
    """
    relative_dir: str = ''
    """
    Relative directory of the manipulator description file in the resources directory.
    """
    gripper_cmd_topic: Optional[str] = None
    """
    Gripper command topic in ROS if it has one.
    """
    gripper_open_cmd_value: Optional[float] = None
    """
    Grip open command value.
    """
    gripper_close_cmd_value: Optional[float] = None
    """
    Grip close command value.
    """
    gripper_relative_dir: Optional[str] = None
    """
    Relative directory of the gripper description file in the resources directory if it has one and is not part of the
     manipulator description file.
    """


def get_point_as_list(point: Point) -> List[float]:
    """
    Return the point as a list.

    :param point: The point.
    :return: The point as a list
    """
    return [point.x, point.y, point.z]


@dataclass
class Color:
    """
    Dataclass for storing rgba_color as an RGBA value.
    The values are stored as floats between 0 and 1.
    The default rgba_color is white. 'A' stands for the opacity.
    """
    R: float = 1
    G: float = 1
    B: float = 1
    A: float = 1

    @classmethod
    def from_list(cls, color: List[float]):
        """
        Set the rgba_color from a list of RGBA values.

        :param color: The list of RGBA values
        """
        if len(color) == 3:
            return cls.from_rgb(color)
        elif len(color) == 4:
            return cls.from_rgba(color)
        else:
            raise ValueError("Color list must have 3 or 4 elements")

    @classmethod
    def from_rgb(cls, rgb: List[float]):
        """
        Set the rgba_color from a list of RGB values.

        :param rgb: The list of RGB values
        """
        return cls(rgb[0], rgb[1], rgb[2], 1)

    @classmethod
    def from_rgba(cls, rgba: List[float]):
        """
        Set the rgba_color from a list of RGBA values.

        :param rgba: The list of RGBA values
        """
        return cls(rgba[0], rgba[1], rgba[2], rgba[3])

    def get_rgba(self) -> List[float]:
        """
        Return the rgba_color as a list of RGBA values.

        :return: The rgba_color as a list of RGBA values
        """
        return [self.R, self.G, self.B, self.A]

    def get_rgb(self) -> List[float]:
        """
        Return the rgba_color as a list of RGB values.

        :return: The rgba_color as a list of RGB values
        """
        return [self.R, self.G, self.B]


class Colors(Color, Enum):
    """
    Enum for easy access to some common colors.
    """
    PINK = (1, 0, 1, 1)
    BLACK = (0, 0, 0, 1)
    WHITE = (1, 1, 1, 1)
    RED = (1, 0, 0, 1)
    GREEN = (0, 1, 0, 1)
    BLUE = (0, 0, 1, 1)
    YELLOW = (1, 1, 0, 1)
    CYAN = (0, 1, 1, 1)
    MAGENTA = (1, 0, 1, 1)
    GREY = (0.5, 0.5, 0.5, 1)

    @classmethod
    def from_string(cls, color: str) -> Color:
        """
        Set the rgba_color from a string. If the string is not a valid color, it will return the color WHITE.

        :param color: The string of the color
        """
        try:
            return cls[color.upper()]
        except KeyError:
            return cls.WHITE


@dataclass
class CollisionCallbacks:
    """
    Dataclass for storing the collision callbacks which are callables that get called when there is a collision
    or when a collision is no longer there.
    """
    on_collision_cb: Callable
    no_collision_cb: Optional[Callable] = None

@dataclass
class LateralFriction:
    """
    Dataclass for storing the information of the lateral friction.
    """
    lateral_friction: float
    lateral_friction_direction: List[float]

@dataclass
class TextAnnotation:
    """
    Dataclass for storing text annotations that can be displayed in the simulation.
    """
    text: str
    position: List[float]
    id: int
    color: Color = field(default_factory=lambda: Color(0, 0, 0, 1))
    size: float = 0.1


@dataclass
class VirtualJoint:
    """
    A virtual (not real) joint that is most likely used for simulation purposes.
    """
    name: str
    type_: JointType
    axes: Optional[Point] = None

    @property
    def type(self):
        return self.type_

    @property
    def is_virtual(self):
        return True

    def __hash__(self):
        return hash(self.name)


class Rotations(Dict[Optional[Union[Grasp, bool]], List[float]]):
    """
    A dictionary that defines standard quaternions for different grasps and orientations. This is mainly used
    to automatically calculate all grasp descriptions of a robot gripper for the robot description.

    SIDE_ROTATIONS: The quaternions for the different approach directions (front, back, left, right)
    VERTICAL_ROTATIONS: The quaternions for the different vertical alignments, in case the object requires for
    example a top grasp
    HORIZONTAL_ROTATIONS: The quaternions for the different horizontal alignments, in case the gripper needs to roll
    90°
    """
    SIDE_ROTATIONS = {
        ApproachDirection.FRONT: [0, 0, 0, 1],
        ApproachDirection.BACK: [0, 0, 1, 0],
        ApproachDirection.LEFT: [0, 0, -math.sqrt(2) / 2, math.sqrt(2) / 2],
        ApproachDirection.RIGHT: [0, 0, math.sqrt(2) / 2, math.sqrt(2) / 2],
    }

    VERTICAL_ROTATIONS = {
        VerticalAlignment.NoAlignment: [0, 0, 0, 1],
        VerticalAlignment.TOP: [0, math.sqrt(2) / 2, 0, math.sqrt(2) / 2],
        VerticalAlignment.BOTTOM: [0, -math.sqrt(2) / 2, 0, math.sqrt(2) / 2],
    }

    HORIZONTAL_ROTATIONS = {
        False: [0, 0, 0, 1],
        True: [math.sqrt(2) / 2, 0, 0, math.sqrt(2) / 2],
    }


@dataclass
class VirtualMobileBaseJoints:
    """
    Dataclass for storing the names, types and axes of the virtual mobile base joints of a mobile robot.
    """

    translation_x: Optional[VirtualJoint] = VirtualJoint(VirtualMobileBaseJointName.LINEAR_X.value,
                                                         JointType.PRISMATIC,
                                                         Point(x=1.0, y=0.0, z=0.0))
    translation_y: Optional[VirtualJoint] = VirtualJoint(VirtualMobileBaseJointName.LINEAR_Y.value,
                                                         JointType.PRISMATIC,
                                                         Point(x=0.0, y=1.0, z=0.0))
    angular_z: Optional[VirtualJoint] = VirtualJoint(VirtualMobileBaseJointName.ANGULAR_Z.value,
                                                     JointType.REVOLUTE,
                                                     Point(x=0.0, y=0.0, z=1.0))

    @property
    def names(self) -> List[str]:
        """
        Return the names of the virtual mobile base joints.
        """
        return [getattr(self, field.name).name for field in fields(self)]

    def get_types(self) -> Dict[str, JointType]:
        """
        Return the joint types of the virtual mobile base joints.
        """
        return {getattr(self, field.name).name: getattr(self, field.name).type_ for field in fields(self)}

    def get_axes(self) -> Dict[str, Point]:
        """
        Return the axes (i.e. The axis on which the joint moves) of the virtual mobile base joints.
        """
        return {getattr(self, field.name).name: getattr(self, field.name).axes for field in fields(self)}


@dataclass
class MultiverseMetaData:
    """Meta data for the Multiverse Client, the simulation_name should be non-empty and unique for each simulation"""
    world_name: str = "world"
    simulation_name: str = "cram"
    length_unit: str = "m"
    angle_unit: str = "rad"
    mass_unit: str = "kg"
    time_unit: str = "s"
    handedness: str = "rhs"
