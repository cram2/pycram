import numpy as np
import pinocchio
from numpy.linalg import norm, solve
from typing_extensions import List, Tuple, Dict

from ..failures import IKError
from ..world_concepts.world_object import Object
from ..datastructures.pose import Pose
from ..ros import logdebug, loginfo

def create_joint_configuration(robot: Object, model: pinocchio.pinocchio_pywrap.Model) -> List[float]:
    joint_to_qindex = dict(zip(model.names, map(lambda joint: joint.idx_q, model.joints)))

    configuration = pinocchio.neutral(model)
    for name, q_idx in joint_to_qindex.items():
        if name == "universe":
            continue
        joint = robot.joints[name]
        configuration[q_idx] = joint.position
    return configuration



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
    joint_to_ids = dict(zip(model.names, map(lambda joint: joint.id, model.joints)))
    joint_to_qindex = dict(zip(model.names, map(lambda joint: joint.idx_q, model.joints)))

    JOINT_ID = model.joints[model.frames[model.getFrameId(target_link)].parent].id
    # Object to destination transformation
    oMdes = pinocchio.SE3(quaternion_rotation_matrix(target_pose.orientation_as_list()), np.array(target_pose.position_as_list()))

    # Initial joint configuration
    q = pinocchio.neutral(model)
    eps = 1e-4
    IT_MAX = 1000
    DT = 1e-1
    damp = 1e-12

    q, success = inverse_kinematics_translation(model, q, data, JOINT_ID, oMdes, eps, IT_MAX, DT, damp)

    if success:
        result = {}
        for name, q_idx in joint_to_qindex.items():
            if name == "universe":
                continue
            result[name] = q[q_idx]

        return result
    else:
        raise IKError("IK failed", target_link, target_link)

def inverse_kinematics_logarithmic(model, configuration, data, target_joint_id, target_transformation, eps=1e-4,
                                   max_iter=1000, dt=1e-1, damp=1e-12) -> Tuple[np.ndarray[float], bool]:
    """
    Compute the inverse kinematics for a given target transformation.

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
    Compute the inverse kinematics for a given target transformation.

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


def clip_joints_to_limits(joint_positions: List[float], joint_limits: List[Tuple[float, float]]) -> np.ndarray[float]:
    """
    Clip the joint positions to the joint limits.

    :param joint_positions: The joint positions to clip.
    :param joint_limits: The joint limits to clip to as a list of tuples (min, max).
    :return: The clipped joint positions.
    """
    return np.array([min(max(joint_position, joint_limit[0]), joint_limit[1]) for joint_position, joint_limit in zip(joint_positions, joint_limits)])



def quaternion_rotation_matrix(quaternion: List[float]) -> np.ndarray:
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    :param quaternion: A 4 element array representing the quaternion (q0,q1,q2,q3)

    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Extract the values from Q, convert from xyzw to wxyz
    q0 = quaternion[3]
    q1 = quaternion[1]
    q2 = quaternion[2]
    q3 = quaternion[0]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix