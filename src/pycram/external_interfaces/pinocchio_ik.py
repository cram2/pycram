from __future__ import annotations

import numpy as np
import pinocchio
from numpy.linalg import norm, solve
from typing_extensions import List, Tuple, Dict

from ..failures import IKError
from ..world_concepts.world_object import Object
from ..datastructures.pose import Pose

def create_joint_configuration(robot: Object, model) -> np.ndarray[float]:
    """
    Create a joint configuration vector (q) from the current joint positions of the robot.
    :param robot: The robot object.
    :param model: The Pinocchio model.
    :return: The joint configuration vector.
    """
    configuration = []
    for name, joint in zip(model.names, model.joints):
        if name == "universe":
            continue
        if joint.shortname() in ["JointModelRUBX", "JointModelRUBY", "JointModelRUBZ"]:
            configuration.append(np.cos(robot.joints[name].position))
            configuration.append(np.sin(robot.joints[name].position))
        else:
            configuration.append(robot.joints[name].position)
    return np.array(configuration)



def compute_ik(target_link: str, target_pose: Pose, robot: Object) -> Dict[str, float]:
    """
    Compute the inverse kinematics for a given target link and pose.

    :param target_link: The target link.
    :param target_pose: The target pose.
    :param robot: The robot object.
    :return: The joint configuration as a dictionary with joint names and joint values.
    """
    model = pinocchio.buildModelFromUrdf(robot.path)
    data = model.createData()

    JOINT_ID = model.frames[model.getFrameId(target_link)].parent
    # Object to destination transformation
    oMdes = pinocchio.XYZQUATToSE3(np.array(target_pose.position_as_list() + target_pose.orientation_as_list()))

    # Initial joint configuration
    # q = pinocchio.neutral(model)
    q = create_joint_configuration(robot, model)
    eps = 1e-4
    IT_MAX = 10000
    DT = 1e-1
    damp = 1e-12

    # q, success = inverse_kinematics_translation(model, q, data, JOINT_ID, oMdes, eps, IT_MAX, DT, damp)
    q, success = inverse_kinematics_logarithmic(model, q, data, JOINT_ID, oMdes, eps, IT_MAX, DT, damp)

    if success:
        return parse_configuration_vector_to_joint_positions(q, model)
    else:
        raise IKError(pinocchio.SE3ToXYZQUAT(oMdes), robot.tf_frame, target_link)

def inverse_kinematics_logarithmic(model, configuration, data, target_joint_id, target_transformation, eps=1e-4,
                                   max_iter=1000, dt=1e-1, damp=1e-12) -> Tuple[np.ndarray[float], bool]:
    """
    Compute the inverse kinematics for a given target transformation. Using a logarithmic error metric.

    :param model: The Pinocchio model.
    :param configuration: The initial joint configuration.
    :param data: The Pinocchio data.
    :param target_joint_id: The target joint ID.
    :param target_transformation: The target transformation.
    :param eps: The error threshold.
    :param max_iter: The maximum number of iterations.
    :param dt: The time step.
    :param damp: The damping factor.
    :return: The final joint configuration.
    """
    q = configuration
    i = 0
    while True:
        pinocchio.forwardKinematics(model, data, q)
        iMd = data.oMi[target_joint_id].actInv(target_transformation)
        err = pinocchio.log(iMd).vector  # in joint frame
        if norm(err) < eps:
            success = True
            break
        if i >= max_iter:
            success = False
            break
        J = pinocchio.computeJointJacobian(model, data, q, target_joint_id)  # in joint frame
        J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pinocchio.integrate(model, q, v * dt)
        q = clip_joints_to_limits(q, list(zip(model.lowerPositionLimit, model.upperPositionLimit)))
        # if not i % 10:
        #     print("%d: error = %s" % (i, err.T))
        i += 1
    return q, success


def inverse_kinematics_translation(model, configuration, data, target_joint_id, target_transformation, eps=1e-4,
                                   max_iter=1000, dt=1e-1, damp=1e-12) -> Tuple[np.ndarray[float], bool]:
    """
    Compute the inverse kinematics for a given target transformation. Using the distance of the translation as error metric.

    :param model: The Pinocchio model.
    :param configuration: The initial joint configuration.
    :param data: The Pinocchio data.
    :param target_joint_id: The target joint ID.
    :param target_transformation: The target transformation.
    :param eps: The error threshold.
    :param max_iter: The maximum number of iterations.
    :param dt: The time step.
    :param damp: The damping factor.
    :return: The final joint configuration.
    """
    q = configuration
    it = 0
    while True:
        pinocchio.forwardKinematics(model, data, q)
        iMd = data.oMi[target_joint_id].actInv(target_transformation)
        err = iMd.translation
        if norm(err) < eps:
            success = True
            break
        if it >= max_iter:
            success = False
            break
        J = pinocchio.computeJointJacobian(model, data, q, target_joint_id)  # in joint frame
        J = -J[:3, :]  # linear part of the Jacobian
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(3), err))
        q = pinocchio.integrate(model, q, v * dt)
        q = clip_joints_to_limits(q, list(zip(model.lowerPositionLimit, model.upperPositionLimit)))

        # if not it % 10:
        #     print("%d: error = %s" % (it, err.T))
        it += 1
    return q, success

def parse_configuration_vector_to_joint_positions(configuration: np.ndarray[float], model) -> Dict[str, float]:
    """
    Takes the configuration vector from pinocchio and the robot model and returns a dictionary with joint names and
    joint values.

    :param configuration: The configuration vector.
    :param model: The robot model.
    :return: The joint configuration as a dictionary with joint names and joint values.
    """
    result = {}
    for joint in model.joints:
        if joint.idx_q == -1:
            continue
        if joint.shortname() in ["JointModelRUBX","JointModelRUBY", "JointModelRUBZ"]:
            # Continuous Joints are represented as sin(theta) and cos(theta) of the joint value, so we use arctan2 to
            # get the joint value
            joint_value = np.arctan2(configuration[joint.idx_q + 1], configuration[joint.idx_q])
        else:
            joint_value = configuration[joint.idx_q]

        result[model.names[joint.id]] = joint_value
    return result


def clip_joints_to_limits(joint_positions: List[float], joint_limits: List[Tuple[float, float]]) -> np.ndarray[float]:
    """
    Clip the joint positions to the joint limits.

    :param joint_positions: The joint positions to clip.
    :param joint_limits: The joint limits to clip to as a list of tuples (min, max).
    :return: The clipped joint positions.
    """
    return np.array([min(max(joint_position, joint_limit[0]), joint_limit[1]) for joint_position, joint_limit in zip(joint_positions, joint_limits)])