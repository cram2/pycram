import tf
import numpy as np
from .bullet_world_reasoning import visible, reachable
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

def height_generator():
    pass


def generate_orientation(position):
    angle = np.arctan2(position[1],position[0]) + np.pi
    quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
    return quaternion

def visibility_validator(pose, robot, object_or_pose, world):
    #Maybe
    robot_pose = robot.get_position_and_orientation()
    if type(object_or_pose) == Object:
        robot.set_position_and_orientation(pose[0], pose[1])
        camera_pose = robot.get_link_position_and_orientation(robot_description.i.get_camera_frame())
        res = visible(object_or_pose, camera_pose, world=world)
    else:
        robot.set_position_and_orientation(pose[0], pose[1])
        #print(pose)
        camera_pose = robot.get_link_position_and_orientation(robot_description.i.get_camera_frame())
        robot.set_position_and_orientation([100, 100, 0], [0, 0, 0, 1])
        print(camera_pose)
        ray = p.rayTest(camera_pose, object_or_pose, physicsClientId=world.client_id)
        print(ray)
        res = ray[0] == -1
    robot.set_position_and_orientation(robot_pose[0], robot_pose[1])
    return res


def reachability_validator(pose, robot, target, gripper, world):
    robot_pose = robot.get_position_and_orientation()
    robot.set_position_and_orientation(pose[0], pose[1])

    res = reachable(target, robot, gripper, world=world)
    robot.set_position_and_orientation(robot_pose[0], robot_pose[1])
    return res
