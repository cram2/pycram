from __future__ import annotations

import numpy as np
import pinocchio
from numpy.linalg import norm, solve
from typing_extensions import List, Tuple, Dict

from ..failures import IKError
from ..world_concepts.world_object import Object
from ..datastructures.pose import PoseStamped
from ..config.ik_conf import PinocchioConfig


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
        # These are continuous joints for all dimensions which are represented by the sin and cos values
        if joint.shortname() in ["JointModelRUBX", "JointModelRUBY", "JointModelRUBZ", "JointModelRevoluteUnboundedUnaligned"]:
            configuration.append(np.cos(robot.joints[name].position))
            configuration.append(np.sin(robot.joints[name].position))
        else:
            # Every other joint of a robot has only one degree of freedom hence they will be directly added to the
            # configuration vector
            configuration.append(robot.joints[name].position)
    return np.array(configuration)


def compute_ik(target_link: str, target_pose: PoseStamped, robot: Object) -> Dict[str, float]:
    """
    Compute the inverse kinematics for a given target link and pose.

    :param target_link: The target link.
    :param target_pose: The target pose.
    :param robot: The robot object.
    :return: The joint configuration as a dictionary with joint names and joint values.
    """
    # Kinematic model of the robot, created form the URDF file
    model = pinocchio.buildModelFromUrdf(robot.path)
    # Mutable data of the robot, includes joint states and position of joints and links
    data = model.createData()

    JOINT_ID = model.frames[model.getFrameId(target_link)].parent
    # Object to destination transformation
    oMdes = pinocchio.XYZQUATToSE3(np.array(target_pose.position.to_list() + target_pose.orientation.to_list()))

    # Initial joint configuration
    # The configuration vector for all joints of the robot, created from the current joint states of the robot
    q = create_joint_configuration(robot, model)
    eps = PinocchioConfig.error_threshold
    IT_MAX = PinocchioConfig.max_iterations
    DT = PinocchioConfig.time_step
    damp = PinocchioConfig.dampening_factor

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
    :return: The final joint configuration and a boolean indicating if the computation was successful.
    """
    q = configuration
    success = False
    for i in range(max_iter):
        # Calculate the current state of all joints for the current iteration
        pinocchio.forwardKinematics(model, data, q)
        # Transformation between the target pose and the current pose of the joint which should reach the target pose
        iMd = data.oMi[target_joint_id].actInv(target_transformation)
        # Error between the current pose and the target pose, this should be minimized
        err = pinocchio.log(iMd).vector  # in joint frame
        # Log is numerically unstable for small errors, so we add a small value to the configuration if the error contains NaN
        if np.isnan(err).any():
            q += 0.0001
        if norm(err) < eps:
            success = True
            break
        J = pinocchio.computeJointJacobian(model, data, q, target_joint_id)  # in joint frame
        J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pinocchio.integrate(model, q, v * dt)
        # Clip all joint values to their limits since the algorithm does not respect joint limits
        q = clip_joints_to_limits(q, model.lowerPositionLimit, model.upperPositionLimit)

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
    :return: The final joint configuration and a boolean indicating if the computation was successful.
    """
    q = configuration
    success = False
    for i in range(max_iter):
        # Calculate the current state of all joints for the current iteration
        pinocchio.forwardKinematics(model, data, q)
        # Transformation between the target pose and the current pose of the joint which should reach the target pose
        iMd = data.oMi[target_joint_id].actInv(target_transformation)
        # Error metric that should be minimized, in this case this is the translation (x, y, z) without the orientation
        err = iMd.translation
        if norm(err) < eps:
            success = True
            break
        J = pinocchio.computeJointJacobian(model, data, q, target_joint_id)  # in joint frame
        J = -J[:3, :]  # linear part of the Jacobian
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(3), err))
        q = pinocchio.integrate(model, q, v * dt)
        # Clip all joint values to their limits since the algorithm does not respect joint limits
        q = clip_joints_to_limits(q, model.lowerPositionLimit, model.upperPositionLimit)

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
        if joint.shortname() in ["JointModelRUBX", "JointModelRUBY", "JointModelRUBZ", "JointModelRevoluteUnboundedUnaligned"]:
            # Continuous Joints are represented as sin(theta) and cos(theta) of the joint value, so we use arctan2 to
            # get the joint value
            joint_value = np.arctan2(configuration[joint.idx_q + 1], configuration[joint.idx_q])
        else:
            joint_value = configuration[joint.idx_q]

        result[model.names[joint.id]] = joint_value
    return result


def clip_joints_to_limits(joint_positions: List[float], lower_limits: List[float], upper_limits: List[float]) -> \
np.ndarray[float]:
    """
    Clip the joint positions to the joint limits.

    :param joint_positions: The joint positions to clip.
    :param lower_limits: A List of lower joint limits.
    :param upper_limits: A List of upper joint limits.
    :return: The clipped joint positions.
    """
    return np.clip(joint_positions, lower_limits, upper_limits)
