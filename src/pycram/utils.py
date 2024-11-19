"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from inspect import isgeneratorfunction
import os
import math

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors as mcolors
from typing_extensions import Tuple, Callable, List, Dict, TYPE_CHECKING

from .datastructures.pose import Pose
from .local_transformer import LocalTransformer


if TYPE_CHECKING:
    from .world_concepts.world_object import Object
    from .robot_description import CameraDescription


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


def _apply_ik(robot: 'Object', pose_and_joint_poses: Tuple[Pose, Dict[str, float]]) -> None:
    """
    Apllies a list of joint poses calculated by an inverse kinematics solver to a robot

    :param robot: The robot the joint poses should be applied on
    :param pose_and_joint_poses: The base pose and joint states as returned by the ik solver
    :return: None
    """
    pose, joint_states = pose_and_joint_poses
    robot.set_pose(pose)
    robot.set_multiple_joint_positions(joint_states)


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


class RayTestUtils:

    def __init__(self, ray_test_batch: Callable, object_id_to_name: Dict = None):
        """
        Initialize the ray test helper.
        """
        self.local_transformer = LocalTransformer()
        self.ray_test_batch = ray_test_batch
        self.object_id_to_name = object_id_to_name

    def get_images_for_target(self, cam_pose: Pose,
                              camera_description: 'CameraDescription',
                              camera_frame: str,
                              size: int = 256,
                              camera_min_distance: float = 0.1,
                              camera_max_distance: int = 3,
                              plot: bool = False) -> List[np.ndarray]:
        """
        Note: The returned color image is a repeated depth image in 3 channels.
        """

        # get the list of start positions of the rays.
        rays_start_positions = self.get_camera_rays_start_positions(camera_description, camera_frame, cam_pose, size,
                                                                    camera_min_distance).tolist()

        # get the list of end positions of the rays
        rays_end_positions = self.get_camera_rays_end_positions(camera_description, camera_frame, cam_pose, size,
                                                                camera_max_distance).tolist()

        # apply the ray test
        object_ids, distances = self.ray_test_batch(rays_start_positions, rays_end_positions, return_distance=True)

        # construct the images/masks
        segmentation_mask = self.construct_segmentation_mask_from_ray_test_object_ids(object_ids, size)
        depth_image = self.construct_depth_image_from_ray_test_distances(distances, size) + camera_min_distance
        color_depth_image = self.construct_color_image_from_depth_image(depth_image)

        if plot:
            self.plot_segmentation_mask(segmentation_mask)
            self.plot_depth_image(depth_image)

        return [color_depth_image, depth_image, segmentation_mask]

    @staticmethod
    def construct_segmentation_mask_from_ray_test_object_ids(object_ids: List[int], size: int) -> np.ndarray:
        """
        Construct a segmentation mask from the object ids returned by the ray test.

        :param object_ids: The object ids.
        :param size: The size of the grid.
        :return: The segmentation mask.
        """
        return np.array(object_ids).squeeze(axis=1).reshape(size, size)

    @staticmethod
    def construct_depth_image_from_ray_test_distances(distances: List[float], size: int) -> np.ndarray:
        """
        Construct a depth image from the distances returned by the ray test.

        :param distances: The distances.
        :param size: The size of the grid.
        :return: The depth image.
        """
        return np.array(distances).reshape(size, size)

    @staticmethod
    def construct_color_image_from_depth_image(depth_image: np.ndarray) -> np.ndarray:
        """
        Construct a color image from the depth image.

        :param depth_image: The depth image.
        :return: The color image.
        """
        min_distance = np.min(depth_image)
        max_distance = np.max(depth_image)
        normalized_depth_image = (depth_image - min_distance) * 255 / (max_distance - min_distance)
        return np.repeat(normalized_depth_image[:, :, np.newaxis], 3, axis=2).astype(np.uint8)

    def get_camera_rays_start_positions(self, camera_description: 'CameraDescription', camera_frame: str,
                                        camera_pose: Pose, size: int,
                                        camera_min_distance: float) -> np.ndarray:

        # get the start pose of the rays from the camera pose and minimum distance.
        start_pose = self.get_camera_rays_start_pose(camera_description, camera_frame, camera_pose, camera_min_distance)

        # get the list of start positions of the rays.
        return np.repeat(np.array([start_pose.position_as_list()]), size * size, axis=0)

    def get_camera_rays_start_pose(self, camera_description: 'CameraDescription', camera_frame: str, camera_pose: Pose,
                                   camera_min_distance: float) -> Pose:
        """
        Get the start position of the camera rays, which is the camera pose shifted by the minimum distance of the
        camera.

        :param camera_description: The camera description.
        :param camera_frame: The camera tf frame.
        :param camera_pose: The camera pose.
        :param camera_min_distance: The minimum distance from which the camera can see.
        """
        camera_pose_in_camera_frame = self.local_transformer.transform_pose(camera_pose, camera_frame)
        start_position = (np.array(camera_description.front_facing_axis) * camera_min_distance
                          + np.array(camera_pose_in_camera_frame.position_as_list()))
        start_pose = Pose(start_position.tolist(), camera_pose_in_camera_frame.orientation_as_list(), camera_frame)
        return self.local_transformer.transform_pose(start_pose, "map")

    def get_camera_rays_end_positions(self, camera_description: 'CameraDescription', camera_frame: str,
                                      camera_pose: Pose, size: int, camera_max_distance: float = 3.0) -> np.ndarray:
        """
        Get the end positions of the camera rays.

        :param camera_description: The camera description.
        :param camera_frame: The camera frame.
        :param camera_pose: The camera pose.
        :param size: The size of the grid.
        :param camera_max_distance: The maximum distance of the camera.
        :return: The end positions of the camera rays.
        """
        rays_horizontal_angles, rays_vertical_angles = self.construct_grid_of_camera_rays_angles(camera_description,
                                                                                                 size)
        rays_end_positions = self.get_end_positions_of_rays_from_angles_and_distance(rays_vertical_angles,
                                                                                     rays_horizontal_angles,
                                                                                     camera_max_distance)
        return self.transform_points_from_camera_frame_to_world_frame(camera_pose, camera_frame, rays_end_positions)

    @staticmethod
    def transform_points_from_camera_frame_to_world_frame(camera_pose: Pose, camera_frame: str,
                                                          points: np.ndarray) -> np.ndarray:
        """
        Transform points from the camera frame to the world frame.

        :param camera_pose: The camera pose.
        :param camera_frame: The camera frame.
        :param points: The points to transform.
        :return: The transformed points.
        """
        cam_to_world_transform = camera_pose.to_transform(camera_frame)
        return cam_to_world_transform.apply_transform_to_array_of_points(points)

    @staticmethod
    def get_end_positions_of_rays_from_angles_and_distance(vertical_angles: np.ndarray, horizontal_angles: np.ndarray,
                                                           distance: float) -> np.ndarray:
        """
        Get the end positions of the rays from the angles and the distance.

        :param vertical_angles: The vertical angles of the rays.
        :param horizontal_angles: The horizontal angles of the rays.
        :param distance: The distance of the rays.
        :return: The end positions of the rays.
        """
        rays_end_positions_x = distance * np.cos(vertical_angles) * np.sin(horizontal_angles)
        rays_end_positions_x = rays_end_positions_x.reshape(-1)
        rays_end_positions_z = distance * np.cos(vertical_angles) * np.cos(horizontal_angles)
        rays_end_positions_z = rays_end_positions_z.reshape(-1)
        rays_end_positions_y = distance * np.sin(vertical_angles)
        rays_end_positions_y = rays_end_positions_y.reshape(-1)
        return np.stack((rays_end_positions_x, rays_end_positions_y, rays_end_positions_z), axis=1)

    @staticmethod
    def construct_grid_of_camera_rays_angles(camera_description: 'CameraDescription',
                                             size: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Construct a 2D grid of camera rays angles.

        :param camera_description: The camera description.
        :param size: The size of the grid.
        :return: The 2D grid of the horizontal and the vertical angles of the camera rays.
        """
        # get the camera fov angles
        camera_horizontal_fov = camera_description.horizontal_angle
        camera_vertical_fov = camera_description.vertical_angle

        # construct a 2d grid of rays angles
        rays_horizontal_angles = np.linspace(-camera_horizontal_fov / 2, camera_horizontal_fov / 2, size)
        rays_horizontal_angles = np.tile(rays_horizontal_angles, (size, 1))
        rays_vertical_angles = np.linspace(-camera_vertical_fov / 2, camera_vertical_fov / 2, size)
        rays_vertical_angles = np.tile(rays_vertical_angles, (size, 1)).T
        return rays_horizontal_angles, rays_vertical_angles

    @staticmethod
    def plot_segmentation_mask(segmentation_mask,
                               object_id_to_name: Dict[int, str] = None):
        """
        Plot the segmentation mask with different colors for each object.

        :param segmentation_mask: The segmentation mask.
        :param object_id_to_name: The mapping from object id to object name.
        """
        if object_id_to_name is None:
            object_id_to_name = {uid: str(uid) for uid in np.unique(segmentation_mask)}

        # Create a custom color map
        unique_ids = np.unique(segmentation_mask)
        unique_ids = unique_ids[unique_ids != -1]  # Exclude -1 values

        # Create a color map that assigns a unique color to each ID
        colors = plt.cm.get_cmap('tab20', len(unique_ids))  # Use tab20 colormap for distinct colors
        color_dict = {uid: colors(i) for i, uid in enumerate(unique_ids)}

        # Map each ID to its corresponding color
        mask_shape = segmentation_mask.shape
        segmentation_colored = np.zeros((mask_shape[0], mask_shape[1], 3))

        for uid in unique_ids:
            segmentation_colored[segmentation_mask == uid] = color_dict[uid][:3]  # Ignore the alpha channel

        # Create a colormap for the color bar
        cmap = mcolors.ListedColormap([color_dict[uid][:3] for uid in unique_ids])
        norm = mcolors.BoundaryNorm(boundaries=np.arange(len(unique_ids) + 1) - 0.5, ncolors=len(unique_ids))

        # Plot the colored segmentation mask
        fig, ax = plt.subplots()
        _ = ax.imshow(segmentation_colored)
        ax.axis('off')  # Hide axes
        ax.set_title('Segmentation Mask with Different Colors for Each Object')

        # Create color bar
        cbar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, ticks=np.arange(len(unique_ids)))
        cbar.ax.set_yticklabels(
            [object_id_to_name[uid] for uid in unique_ids])  # Label the color bar with object IDs
        cbar.set_label('Object Name')

        plt.show()

    @staticmethod
    def plot_depth_image(depth_image):
        # Plot the depth image
        fig, ax = plt.subplots()
        cax = ax.imshow(depth_image, cmap='viridis', vmin=0, vmax=np.max(depth_image))
        ax.axis('off')  # Hide axes
        ax.set_title('Depth Image')

        # Create color bar
        cbar = fig.colorbar(cax, ax=ax)
        cbar.set_label('Depth Value')

        plt.show()


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

