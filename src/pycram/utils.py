"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from __future__ import annotations

from copy import deepcopy
from inspect import isgeneratorfunction
import os
import math
from typing import Union, Iterator

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors as mcolors
from semantic_digital_twin.world_description.world_entity import Body

from .tf_transformations import quaternion_about_axis, quaternion_multiply, quaternion_matrix
from typing_extensions import Tuple, Callable, List, Dict, TYPE_CHECKING, Sequence, Any, Iterable

from .datastructures.pose import PoseStamped

if TYPE_CHECKING:
    from .robot_description import CameraDescription



def link_pose_for_joint_config(
        obj: Body,
        joint_config: Dict[str, float]) -> PoseStamped:
    """
    Get the pose a link would be in if the given joint configuration would be applied to the object.
    This is done by using the respective object in the prospection world and applying the joint configuration
    to this one. After applying the joint configuration the link position is taken from there.

    :param obj: The body for which the pose should be calculated
    :param joint_config: Dict with the goal joint configuration
    :return: The pose of the link after applying the joint configuration
    """
    reasoning_world = deepcopy(obj._world)
    for joint_name, joint_pose in joint_config.items():
        reasoning_world.state[reasoning_world.get_degree_of_freedom_by_name(joint_name).name].position = joint_pose
    reasoning_world.notify_state_change()
    pose = reasoning_world.get_body_by_name(obj.name).global_pose
    return PoseStamped.from_spatial_type(pose)


def get_rays_from_min_max(min_bound: Sequence[float], max_bound: Sequence[float], step_size_in_meters: float = 0.01) \
        -> np.ndarray:
    """
    Get rays from min and max bounds as an array of start and end 3D points.
    Note: The rays are not steped in the x direction as the rays are cast parallel to the x-axis.

    Example:
    >>> min_bound = [0, 0, 0]
    >>> max_bound = [1, 2, 3]
    >>> rays = get_rays_from_min_max(min_bound, max_bound, 1)
    >>> rays.shape
    (6, 3, 2)
    >>> rays
    array([
    [[0. , 1. ],
     [0. , 0. ],
     [0. , 0. ]],
    [[0. , 1. ],
     [0. , 0. ],
     [1.5, 1.5]],
    [[0. , 1. ],
     [0. , 0. ],
     [3. , 3. ]],
    [[0. , 1. ],
     [2. , 2. ],
     [0. , 0. ]],
    [[0. , 1. ],
     [2. , 2. ],
     [1.5, 1.5]],
    [[0. , 1. ],
     [2. , 2. ],
     [3. , 3. ]]
     ])

    :param min_bound: The minimum bound of the rays, a sequence of 3 floats.
    :param max_bound: The maximum bound of the rays, a sequence of 3 floats.
    :param step_size_in_meters: The step size in meters between the rays.
    :return: The rays as an array of shape (n, 3, 2) where n is number of rays, 3 is because each point has x, y, and z,
    and 2 is for the start and end points of the rays.
    """
    min_bound = np.array(min_bound)
    max_bound = np.array(max_bound)
    n_steps = np.ceil(np.abs(max_bound[1:] - min_bound[1:]) / step_size_in_meters).astype(int)
    rays_start_x = np.ones((n_steps[0], n_steps[1])) * min_bound[0]
    rays_end_x = np.ones((n_steps[0], n_steps[1])) * max_bound[0]
    y_values = np.linspace(min_bound[1], max_bound[1], n_steps[0])
    z_values = np.linspace(min_bound[2], max_bound[2], n_steps[1])
    rays_start_y = np.tile(y_values, (n_steps[1], 1)).T
    rays_end_y = rays_start_y
    rays_start_z = np.tile(z_values, (n_steps[0], 1))
    rays_end_z = rays_start_z
    rays_start = np.stack((rays_start_x, rays_start_y, rays_start_z), axis=-1)
    rays_end = np.stack((rays_end_x, rays_end_y, rays_end_z), axis=-1)
    rays_start = rays_start.reshape(-1, 3)
    rays_end = rays_end.reshape(-1, 3)
    # The shape of rays is (num_rays, 3, 2), while num_rays = n_steps[0] (num y step) * n_steps[1] (num z step)
    return np.stack((rays_start, rays_end), axis=-1)


def chunks(lst: Union[List, np.ndarray], n: int) -> Iterator[List]:
    """
    Yield successive n-sized chunks from lst.

    :param lst: The list from which chunks should be yielded
    :param n: Size of the chunks
    :return: A list of size n from lst
    """
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


class bcolors:
    """
    Color codes which can be used to highlight Text in the Terminal. For example,
    for warnings.
    Usage:
    Firstly import the class into the file.
    print(f'{bcolors.WARNING} Some Text {bcolors.ENDC}')
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class GeneratorList:
    """Implementation of generator list wrappers.

    Generator lists store the elements of a generator, so these can be fetched multiple times.

    Methods:
    get -- get the element at a specific index.
    has -- check if an element at a specific index exists.
    """

    def __init__(self, generator: Callable):
        """Create a new generator list.

        Arguments:
        generator -- the generator to use.
        """
        if isgeneratorfunction(generator):
            self._generator = generator()
        else:
            self._generator = generator

        self._generated = []

    def get(self, index: int = 0):
        """Get the element at a specific index or raise StopIteration if it doesn't exist.

        Arguments:
        index -- the index to get the element of.
        """
        while len(self._generated) <= index:
            self._generated.append(next(self._generator))

        return self._generated[index]

    def has(self, index: int) -> bool:
        """Check if an element at a specific index exists and return True or False.

        Arguments:
        index -- the index to check for.
        """
        try:
            self.get(index)
            return True
        except StopIteration:
            return False


def axis_angle_to_quaternion(axis: List, angle: float) -> Tuple:
    """
    Convert axis-angle to quaternion.

    :param axis: (x, y, z) tuple representing rotation axis.
    :param angle: rotation angle in degree
    :return: The quaternion representing the axis angle
    """
    angle = math.radians(angle)
    axis_length = math.sqrt(sum([i ** 2 for i in axis]))
    normalized_axis = tuple(i / axis_length for i in axis)

    x = normalized_axis[0] * math.sin(angle / 2)
    y = normalized_axis[1] * math.sin(angle / 2)
    z = normalized_axis[2] * math.sin(angle / 2)
    w = math.cos(angle / 2)

    return tuple((x, y, z, w))


class suppress_stdout_stderr(object):
    """
    A context manager for doing a "deep suppression" of stdout and stderr in
    Python, i.e. will suppress all prints, even if the print originates in a
    compiled C/Fortran sub-function.

    This will not suppress raised exceptions, since exceptions are printed
    to stderr just before a script exits, and after the context manager has
    exited (at least, I think that is why it lets exceptions through).
    Copied from https://stackoverflow.com/questions/11130156/suppress-stdout-stderr-print-from-python-functions
    """

    def __init__(self):
        # Open a pair of null files
        self.null_fds = [os.open(os.devnull, os.O_RDWR) for _ in range(2)]
        # Save the actual stdout (1) and stderr (2) file descriptors.
        self.save_fds = [os.dup(1), os.dup(2)]

    def __enter__(self):
        # Assign the null pointers to stdout and stderr.
        # This one is not needed for URDF parsing output
        # os.dup2(self.null_fds[0], 1)
        os.dup2(self.null_fds[1], 2)

    def __exit__(self, *_):
        # Re-assign the real stdout/stderr back to (1) and (2)
        # This one is not needed for URDF parsing output
        # os.dup2(self.save_fds[0], 1)
        os.dup2(self.save_fds[1], 2)
        # Close all file descriptors
        for fd in self.null_fds + self.save_fds:
            os.close(fd)


def adjust_camera_pose_based_on_target(cam_pose: PoseStamped, target_pose: PoseStamped,
                                       camera_description: CameraDescription) -> PoseStamped:
    """
    Adjust the given cam_pose orientation such that it is facing the target_pose, which partly depends on the
     front_facing_axis of the that is defined in the camera_description.

    :param cam_pose: The camera pose.
    :param target_pose: The target pose.
    :param camera_description: The camera description.
    :return: The adjusted camera pose.
    """
    quaternion = get_quaternion_between_camera_and_target(cam_pose, target_pose, camera_description)
    # apply the rotation to the camera pose using quaternion multiplication
    return apply_quaternion_to_pose(cam_pose, quaternion)


def get_quaternion_between_camera_and_target(cam_pose: PoseStamped, target_pose: PoseStamped,
                                             camera_description: 'CameraDescription') -> np.ndarray:
    """
    Get the quaternion between the camera and the target.

    :param cam_pose: The camera pose.
    :param target_pose: The target pose.
    :param camera_description: The camera description.
    :return: The quaternion between the camera and the target.
    """
    # Get the front facing axis of the camera in the world frame
    front_facing_axis = transform_vector_using_pose(camera_description.front_facing_axis, cam_pose)
    front_facing_axis = front_facing_axis - np.array(cam_pose.position.to_list())

    # Get the vector from the camera to the target
    camera_to_target = cam_pose.get_vector_to_pose(target_pose)

    # Get the quaternion between the camera and target
    return get_quaternion_between_two_vectors(front_facing_axis, camera_to_target)


def transform_vector_using_pose(vector: Sequence, pose) -> np.ndarray:
    """
    Transform a vector using a pose.

    :param vector: The vector.
    :param pose: The pose.
    :return: The transformed vector.
    """
    vector = np.array(vector).reshape(1, 3)
    return pose.to_transform_stamped("pose").apply_transform_to_array_of_points(vector).flatten()


def apply_quaternion_to_pose(pose: PoseStamped, quaternion: np.ndarray) -> PoseStamped:
    """
    Apply a quaternion to a pose.

    :param pose: The pose.
    :param quaternion: The quaternion.
    :return: The new pose.
    """
    pose_quaternion = np.array(pose.orientation.to_list())
    new_quaternion = quaternion_multiply(quaternion, pose_quaternion)
    return PoseStamped(pose.position.to_list(), new_quaternion.tolist())


def get_quaternion_between_two_vectors(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
    """
    Get the quaternion between two vectors.

    :param v1: The first vector.
    :param v2: The second vector.
    :return: The quaternion between the two vectors.
    """
    axis, angle = get_axis_angle_between_two_vectors(v1, v2)
    return quaternion_about_axis(angle, axis)


def get_axis_angle_between_two_vectors(v1: np.ndarray, v2: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Get the axis and angle between two vectors.

    :param v1: The first vector.
    :param v2: The second vector.
    :return: The axis and angle between the two vectors.
    """
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    axis = np.cross(v1, v2)
    angle = np.arccos(np.dot(v1, v2) - 1e-9)  # to avoid numerical errors
    return axis, angle


def wxyz_to_xyzw(wxyz: List[float]) -> List[float]:
    """
    Convert a quaternion from WXYZ to XYZW format.
    """
    return [wxyz[1], wxyz[2], wxyz[3], wxyz[0]]


def xyzw_to_wxyz(xyzw: List[float]) -> List[float]:
    """
    Convert a quaternion from XYZW to WXYZ format.

    :param xyzw: The quaternion in XYZW format.
    """
    return [xyzw[3], *xyzw[:3]]


def wxyz_to_xyzw_arr(wxyz: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion from WXYZ to XYZW format.

    :param wxyz: The quaternion in WXYZ format.
    """
    xyzw = np.zeros(4)
    xyzw[:3] = wxyz[1:]
    xyzw[3] = wxyz[0]
    return xyzw


def xyzw_to_wxyz_arr(xyzw: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion from XYZW to WXYZ format.

    :param xyzw: The quaternion in XYZW format.
    """
    wxyz = np.zeros(4)
    wxyz[0] = xyzw[3]
    wxyz[1:] = xyzw[:3]
    return wxyz



class ClassPropertyDescriptor:
    """
    A helper that can be used to define properties of a class like the built-in ones but does not require the class
    to be instantiated.
    """

    def __init__(self, fget, fset=None):
        self.fget = fget
        self.fset = fset

    def __get__(self, obj, klass=None):
        if klass is None:
            klass = type(obj)
        return self.fget.__get__(obj, klass)()

    def __set__(self, obj, value):
        if not self.fset:
            raise AttributeError("can't set attribute")
        type_ = type(obj)
        return self.fset.__get__(obj, type_)(value)

    def setter(self, func):
        if not isinstance(func, (classmethod, staticmethod)):
            func = classmethod(func)
        self.fset = func
        return self


def classproperty(func):
    if not isinstance(func, (classmethod, staticmethod)):
        func = classmethod(func)

    return ClassPropertyDescriptor(func)


def is_iterable(obj: Any) -> bool:
    """
    Checks if the given object is iterable.

    :param obj: The object that should be checked
    :return: True if the object is iterable, False otherwise
    """
    try:
        iter(obj)
    except TypeError:
        return False
    return True


def lazy_product(*iterables: Iterable, iter_names: List[str] = None) -> Iterable[Tuple]:
    """
    Lazily generate the cartesian product of the iterables.

    :param iterables: Iterable of iterables to construct product for.
    :param iter_names: Optional names for the iterables for better error messages.
    :return: Iterable of tuples in the cartesian product.
    """

    consumable_iterables = [iter(iterable) for iterable in iterables]

    current_value = []
    for i, consumable_iterable in enumerate(consumable_iterables):
        try:
            current_value.append(next(consumable_iterable))
        except StopIteration as e:
            raise RuntimeError(f"No values in the iterable: {consumable_iterable} for iterable '{iter_names[i] if iter_names else i}'")

    while True:
        yield tuple(current_value)

        for index in range(len(consumable_iterables) -1, -1, -1):
            current_iterable = consumable_iterables[index]
            try:
                consumable_value = next(current_iterable)
                current_value[index] = consumable_value
                break
            except StopIteration as e:
                if index == 0:
                    return
                consumable_iterables[index] = iter(iterables[index])
                try:
                    current_value[index] = next(consumable_iterables[index])
                except StopIteration as e:
                    raise StopIteration(f"No more values in the iterable: {iterables[index]}")


def translate_pose_along_local_axis(pose: PoseStamped, axis: Union[List, np.ndarray], distance: float) -> PoseStamped:
    """
    Translate a pose along a given 3d vector (axis) by a given distance. The axis is given in the local coordinate
    frame of the pose. The axis is normalized and then scaled by the distance.

    :param pose: The pose that should be translated
    :param axis: The local axis along which the translation should be performed
    :param distance: The distance by which the pose should be translated

    :return: The translated pose
    """
    normalized_translation_vector = np.array(axis) / np.linalg.norm(axis)

    rot_matrix = quaternion_matrix(pose.orientation.to_list())[:3, :3]
    translation_in_world = rot_matrix @ normalized_translation_vector
    scaled_translation_vector = np.array(pose.position.to_list()) + translation_in_world * distance

    return PoseStamped.from_list(list(scaled_translation_vector), pose.orientation.to_list(), pose.frame_id )