from datetime import datetime
from functools import lru_cache

from ..designators.action_designator import *
from ..designators.object_designator import *
from ..plan import ActionNode, DesignatorNode, ResolvedActionNode, PlanNode
from ..ros import create_publisher
from ..ros import Time as ROSTime

try:
    from knowrob_designator.msg import DesignatorExecutionFinished, DesignatorExecutionStart, DesignatorInit, \
        DesignatorResolutionFinished, DesignatorResolutionStart, ObjectDesignator
except ImportError:
    loginfo("Could not import knowrob_designator.msg")

desig_execution_start = create_publisher("knowrob/designator_execution_started", DesignatorExecutionStart)
desig_execution_finished = create_publisher("knowrob/designator_execution_finished", DesignatorExecutionFinished)
desig_resolution_start = create_publisher("knowrob/designator_resolving_started", DesignatorResolutionStart)
desig_resolution_finished = create_publisher("knowrob/designator_resolving_finished", DesignatorResolutionFinished)
desig_init = create_publisher("knowrob/designator_init", DesignatorInit)
object_desig = create_publisher("knowrob/object_designator", ObjectDesignator)

@lru_cache(maxsize=None)
def init_object_state():
    """
    Publishes the initial state of the objects in the world to Knowrob. Because of the lru_cache decorator, this function
    will only be called once, and the result will be cached for future calls.
    """
    for obj in World.current_world.objects:
        obj_json = {"anObject": {"type": str(obj.obj_type)}}

        msg = ObjectDesignator()
        msg.json_designator = str(obj_json)
        split_time = str(datetime.now().timestamp()).split(".")
        msg.start = ROSTime(int(split_time[0]), int(split_time[1]))
        object_desig.publish(msg)


def pose_to_json(pose: PoseStamped) -> Dict[str, Union[float, str]]:
    """
    Converts a PoseStamped object to a JSON-like dictionary.

    :return : A dictionary representation of the pose.
    """
    return {"px": pose.position.x, "py": pose.position.y, "pz": pose.position.z, "rx": pose.orientation.x,
            "ry": pose.orientation.y, "rz": pose.orientation.z, "rw": pose.orientation.w, "frame": pose.header.frame_id}


def object_to_json(obj: Union[ObjectDesignator, Object]) -> Dict[str, Dict[str, str]]:
    """
    Converts an object or object designator to a JSON-like dictionary containing its type.

    :param obj: The object or object designator to convert.
    :return: A dictionary representation of the object.
    """
    if isinstance(obj, ObjectDesignatorDescription):
        return {"anObject": {"type": str(obj.types[0])}}
    else:
        return {"anObject": {"type": str(obj.obj_type)}}


def grasp_description_to_json(grasp: GraspDescription) -> Dict[str, Union[float, str]]:
    """
    Converts a GraspDescription object to a JSON-like dictionary.

    :param grasp: The GraspDescription object to convert.
    :return: A dictionary representation of the grasp.
    """
    return {"approach_direction": grasp.approach_direction, "vertical_alignment": grasp.vertical_alignment,
            "rotate_gripper": grasp.rotate_gripper}


def kwargs_to_json(kwargs: Dict) -> dict:
    """
    Converts the keyword arguments of a DesignatorNode to a JSON-like dictionary, calls the appropriate conversion function
    for each value based on its type.

    :param kwargs: The keyword arguments to convert.
    :return: A dictionary representation of the keyword arguments.
    """
    res = {}
    for name, value in kwargs.items():
        if isinstance(value, PoseStamped):
            res[name] = pose_to_json(value)
        elif isinstance(value, Object) or isinstance(value, ObjectDesignatorDescription):
            res[name] = object_to_json(value)
        elif isinstance(value, GraspDescription):
            res[name] = grasp_description_to_json(value)
        else:
            res[name] = value
    return res


def designator_to_json(node: DesignatorNode) -> str:
    """
    Converts a DesignatorNode to a JSON-like string representation uses the kwargs of the node for conversion.

    :param node: The DesignatorNode to convert.
    :return: A JSON-like string representation of the DesignatorNode.
    """
    params = {"type": str(node.action.__name__)}
    params.update(kwargs_to_json(node.kwargs))
    params = {"anAction": params}
    return str(params)

def get_current_ros_time() -> ROSTime:
    """
    Returns the ROS time as a ROSTime object.

    :return: The current ROS time.
    """
    split_time = str(datetime.now().timestamp()).split(".")
    return ROSTime(int(split_time[0]), int(split_time[1]))


def execution_start_callback(node: PlanNode):
    """
    Callback function that is called when the execution of a designator starts. It publishes a message to Knowrob
    containing the designator that is executed.
    """
    msg = DesignatorExecutionStart()
    msg.designator_id = str(hash(node))
    msg.json_designator = designator_to_json(node)
    msg.stamp = get_current_ros_time()

    desig_execution_start.publish(msg)


def execution_finished_callback(node: PlanNode):
    """
    Callback function that is called when the execution of a designator finishes. It publishes a message to Knowrob
    containing the designator that has just finished executing.
    """
    msg = DesignatorExecutionFinished()
    msg.designator_id = str(hash(node))
    msg.json_designator = designator_to_json(node)
    msg.stamp = get_current_ros_time()

    desig_execution_finished.publish(msg)


def resolution_start_callback(node: PlanNode):
    """
    Callback function that is called when the resolution of a designator starts. It publishes a message to Knowrob
    containing the designator that is being resolved. It also sends the initial state of the objects in the world.
    """
    init_object_state()

    msg = DesignatorResolutionStart()
    msg.designator_id = str(hash(node))
    msg.json_designator = designator_to_json(node)
    msg.stamp = get_current_ros_time()

    desig_resolution_start.publish(msg)


def resolution_finished_callback(node: PlanNode):
    """
    Callback function that is called when the resolution of a designator finishes. It publishes a message to Knowrob
    containing the designator that has just finished resolving.
    """
    msg = DesignatorResolutionFinished()
    msg.designator_id = str(hash(node.children[-1]))
    msg.json_designator = designator_to_json(node)
    msg.stamp = get_current_ros_time()
    msg.resolved_from_id = str(hash(node))

    desig_resolution_finished.publish(msg)


# Registers the callback to the Type of Action

Plan.add_on_start_callback(resolution_start_callback, ActionNode)
Plan.add_on_start_callback(execution_start_callback, ResolvedActionNode)

Plan.add_on_end_callback(resolution_finished_callback, ActionNode)
Plan.add_on_end_callback(execution_finished_callback, ResolvedActionNode)
