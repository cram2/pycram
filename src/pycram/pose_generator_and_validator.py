import tf
import numpy as np
import .bullet_world_reasoning as btr
import pybullet as p
from .bullet_world import Object
from .robot_description import InitializedRobotDescription as robot_description

def pose_generator(costmap):
    indices = np.argpartition(costmap.map.flatten(), -100)[-100:]
    indices = np.dstack(np.unravel_index(indices, costmap.map.shape)).reshape(100, 2)
    size = costmap.map.shape[0]
    for ind in indices:
        position = [(ind[0]- size/2) * costmap.resolution, (ind[1] - size/2) * costmap.resolution, 0]
        orientation = generate_orientation(position)
        yield (list(position), orientation)


def generate_orientation(position):
    angle = np.arctan2(position[1],position[0]) + np.pi
    quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
    return quaternion

def visibility_validator(pose, robot, object_or_pose):
    #Maybe
    robot.set_position_and_orientation(pose)
    camera_pose = robot.get_link_position_and_orientation(robot_description.get_camera_frame())
    if type(object_or_pose) == Object:
        return btr.visibile(object_or_pose, camera_pose)
    else:
        ray = p.rayTest(camera_pose, object_or_pose)
        return ray[0] == -1


def reachability_validator(pose, robot, target, gripper):
    robot.set_position_and_orientation(pose)
    return btr.reachable(target, robot, gripper)
