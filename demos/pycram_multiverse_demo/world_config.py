from __future__ import annotations

import abc
from abc import ABC
from typing import Dict, Optional, Union

import numpy as np
import rospy
from numpy.typing import NDArray
from std_msgs.msg import ColorRGBA

from giskardpy.god_map import god_map
from giskardpy.model.joints import FixedJoint, OmniDrive, DiffDrive, Joint6DOF, OneDofJoint
from giskardpy.model.links import Link
from giskardpy.model.utils import robot_name_from_urdf_string
from giskardpy.model.world import WorldTree
from giskardpy.data_types import my_string, PrefixName, Derivatives, derivative_map


class WorldConfig(ABC):

    def __init__(self):
        god_map.world = WorldTree()
        self.set_default_weights()

    @property
    def world(self) -> WorldTree:
        return god_map.world

    def set_defaults(self):
        pass

    @abc.abstractmethod
    def setup(self):
        """
        Implement this method to configure the initial world using it's self. methods.
        """

    @property
    def robot_group_name(self) -> str:
        return god_map.world.robot_name

    def set_default_weights(self,
                            velocity_weight: float = 0.01,
                            acceleration_weight: float = 0,
                            jerk_weight: float = 0.01):
        """
        The default values are set automatically, even if this function is not called.
        A typical goal has a weight of 1, so the values in here should be sufficiently below that.
        """
        god_map.world.update_default_weights({Derivatives.velocity: velocity_weight,
                                               Derivatives.acceleration: acceleration_weight,
                                               Derivatives.jerk: jerk_weight})

    def set_weight(self, weight_map: derivative_map, joint_name: str, group_name: Optional[str] = None):
        """
        Set weights for joints that are used by the qp controller. Don't change this unless you know what you are doing.
        """
        joint_name = god_map.world.search_for_joint_name(joint_name, group_name)
        joint = god_map.world.joints[joint_name]
        if not isinstance(joint, OneDofJoint):
            raise ValueError(f'Can\'t change weight because {joint_name} is not of type {str(OneDofJoint)}.')
        free_variable = god_map.world.free_variables[joint.free_variable.name]
        for derivative, weight in weight_map.items():
            free_variable.quadratic_weights[derivative] = weight

    def get_root_link_of_group(self, group_name: str) -> PrefixName:
        return god_map.world.groups[group_name].root_link_name

    def set_joint_limits(self, limit_map: derivative_map, joint_name: my_string, group_name: Optional[str] = None):
        """
        Set the joint limits for individual joints
        :param limit_map: maps Derivatives to values, e.g. {Derivatives.velocity: 1,
                                                            Derivatives.acceleration: np.inf,
                                                            Derivatives.jerk: 30}
        """
        joint_name = god_map.world.search_for_joint_name(joint_name, group_name)
        joint = god_map.world.joints[joint_name]
        if not isinstance(joint, OneDofJoint):
            raise ValueError(f'Can\'t change limits because {joint_name} is not of type {str(OneDofJoint)}.')
        free_variable = god_map.world.free_variables[joint.free_variable.name]
        for derivative, limit in limit_map.items():
            free_variable.set_lower_limit(derivative, -limit)
            free_variable.set_upper_limit(derivative, limit)

    def set_default_color(self, r: float, g: float, b: float, a: float):
        """
        :param r: 0-1
        :param g: 0-1
        :param b: 0-1
        :param a: 0-1
        """
        god_map.world.default_link_color = ColorRGBA(r, g, b, a)

    def set_default_limits(self, new_limits: derivative_map):
        """
        The default values will be set automatically, even if this function is not called.
        :param new_limits: e.g. {Derivatives.velocity: 1,
                                 Derivatives.acceleration: np.inf,
                                 Derivatives.jerk: 30}
        """
        god_map.world.update_default_limits(new_limits)

    def add_robot_urdf(self,
                       urdf: str,
                       group_name: str) -> str:
        """
        Add a robot urdf to the world.
        :param urdf: robot urdf as string, not the path
        :param group_name:
        """
        if group_name is None:
            group_name = robot_name_from_urdf_string(urdf)
        god_map.world.add_urdf(urdf=urdf, group_name=group_name, actuated=True)
        return group_name

    def add_robot_from_parameter_server(self,
                                        parameter_name: str = 'robot_description',
                                        group_name: Optional[str] = None) -> str:
        """
        Add a robot urdf from parameter server to Giskard.
        :param parameter_name:
        :param group_name: How to call the robot. If nothing is specified it will get the name it has in the urdf
        """
        urdf = rospy.get_param(parameter_name)
        return self.add_robot_urdf(urdf=urdf, group_name=group_name)

    def add_fixed_joint(self, parent_link: my_string, child_link: my_string,
                        homogenous_transform: Optional[NDArray] = None):
        """
        Add a fixed joint to Giskard's world. Can be used to e.g. connect a non-mobile robot to the world frame.
        :param parent_link:
        :param child_link:
        :param homogenous_transform: a 4x4 transformation matrix.
        """
        if homogenous_transform is None:
            homogenous_transform = np.eye(4)
        parent_link = god_map.world.search_for_link_name(parent_link)

        child_link = PrefixName.from_string(child_link, set_none_if_no_slash=True)
        joint_name = PrefixName(f'{parent_link}_{child_link}_fixed_joint', None)
        joint = FixedJoint(name=joint_name, parent_link_name=parent_link, child_link_name=child_link,
                           parent_T_child=homogenous_transform)
        god_map.world._add_joint(joint)

    def add_diff_drive_joint(self,
                             name: str,
                             parent_link_name: my_string,
                             child_link_name: my_string,
                             robot_group_name: Optional[str] = None,
                             translation_limits: Optional[derivative_map] = None,
                             rotation_limits: Optional[derivative_map] = None):
        """
        Same as add_omni_drive_joint, but for a differential drive.
        """
        joint_name = PrefixName(name, robot_group_name)
        parent_link_name = PrefixName.from_string(parent_link_name, set_none_if_no_slash=True)
        child_link_name = PrefixName.from_string(child_link_name, set_none_if_no_slash=True)
        brumbrum_joint = DiffDrive(parent_link_name=parent_link_name,
                                   child_link_name=child_link_name,
                                   name=joint_name,
                                   translation_limits=translation_limits,
                                   rotation_limits=rotation_limits)
        god_map.world._add_joint(brumbrum_joint)
        god_map.world.deregister_group(robot_group_name)
        god_map.world.register_group(robot_group_name, root_link_name=parent_link_name, actuated=True)

    def add_6dof_joint(self, parent_link: my_string, child_link: my_string, joint_name: my_string):
        """
        Add a 6dof joint to Giskard's world. Generally used if you want Giskard to keep track of a tf transform,
        e.g. for localization.
        :param parent_link:
        :param child_link:
        """
        parent_link = god_map.world.search_for_link_name(parent_link)
        child_link = PrefixName.from_string(child_link, set_none_if_no_slash=True)
        joint_name = PrefixName.from_string(joint_name, set_none_if_no_slash=True)
        joint = Joint6DOF(name=joint_name, parent_link_name=parent_link, child_link_name=child_link)
        god_map.world._add_joint(joint)

    def add_empty_link(self, link_name: my_string):
        """
        If you need a virtual link during your world building.
        """
        link = Link(link_name)
        god_map.world._add_link(link)

    def add_omni_drive_joint(self,
                             name: str,
                             parent_link_name: Union[str, PrefixName],
                             child_link_name: Union[str, PrefixName],
                             robot_group_name: Optional[str] = None,
                             translation_limits: Optional[derivative_map] = None,
                             rotation_limits: Optional[derivative_map] = None,
                             x_name: Optional[PrefixName] = None,
                             y_name: Optional[PrefixName] = None,
                             yaw_vel_name: Optional[PrefixName] = None):
        """
        Use this to connect a robot urdf of a mobile robot to the world if it has an omni-directional drive.
        :param parent_link_name:
        :param child_link_name:
        :param robot_group_name: set if there are multiple robots
        :param name: Name of the new link. Has to be unique and may be required in other functions.
        :param translation_limits: in m/s**3
        :param rotation_limits: in rad/s**3
        """
        joint_name = PrefixName(name, robot_group_name)
        parent_link_name = PrefixName.from_string(parent_link_name, set_none_if_no_slash=True)
        child_link_name = PrefixName.from_string(child_link_name, set_none_if_no_slash=True)
        brumbrum_joint = OmniDrive(parent_link_name=parent_link_name,
                                   child_link_name=child_link_name,
                                   name=joint_name,
                                   translation_limits=translation_limits,
                                   rotation_limits=rotation_limits,
                                   x_name=x_name,
                                   y_name=y_name,
                                   yaw_name=yaw_vel_name)
        god_map.world._add_joint(brumbrum_joint)
        god_map.world.deregister_group(robot_group_name)
        god_map.world.register_group(robot_group_name, root_link_name=parent_link_name, actuated=True)


class EmptyWorld(WorldConfig):
    def __init__(self):
        super().__init__()

    def setup(self):
        self._default_limits = {
            Derivatives.velocity: 1,
        }
        self.set_default_weights(velocity_weight=1, acceleration_weight=1, jerk_weight=1)
        self.set_default_limits(self._default_limits)
        self.add_empty_link('map')


class WorldWithFixedRobot(WorldConfig):
    def __init__(self, joint_limits: Dict[Derivatives, float] = None):
        super().__init__()
        self._joint_limits = joint_limits

    def setup(self):
        self.set_default_limits(self._joint_limits)
        self.add_robot_from_parameter_server()


class WorldWithOmniDriveRobot(WorldConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum'):
        super().__init__()
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.set_default_limits({Derivatives.velocity: 1,
                                 Derivatives.acceleration: np.inf,
                                 Derivatives.jerk: 30})
        self.add_empty_link(self.map_name)
        self.add_empty_link(self.odom_link_name)
        self.add_fixed_joint(parent_link=self.map_name, child_link=self.odom_link_name)
        # self.add_6dof_joint(parent_link=self.map_name, child_link=self.odom_link_name,
        #                     joint_name=self.localization_joint_name)
        self.add_robot_from_parameter_server()
        root_link_name = self.get_root_link_of_group(self.robot_group_name)
        self.add_omni_drive_joint(name=self.drive_joint_name,
                                  parent_link_name=self.odom_link_name,
                                  child_link_name=root_link_name,
                                  translation_limits={
                                      Derivatives.velocity: 0.2,
                                      Derivatives.acceleration: 1,
                                      Derivatives.jerk: 5,
                                  },
                                  rotation_limits={
                                      Derivatives.velocity: 0.2,
                                      Derivatives.acceleration: 1,
                                      Derivatives.jerk: 5
                                  },
                                  robot_group_name=self.robot_group_name)
        self.set_joint_limits(limit_map={Derivatives.velocity: 3,
                                         Derivatives.jerk: 60},
                              joint_name='head_pan_joint')


class WorldWithDiffDriveRobot(WorldConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum'):
        super().__init__()
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.set_default_limits({Derivatives.velocity: 1,
                                              Derivatives.acceleration: np.inf,
                                              Derivatives.jerk: 30})
        self.add_empty_link(self.map_name)
        self.add_empty_link(self.odom_link_name)
        self.add_6dof_joint(parent_link=self.map_name, child_link=self.odom_link_name,
                                         joint_name=self.localization_joint_name)
        self.add_robot_from_parameter_server()
        root_link_name = self.get_root_link_of_group(self.robot_group_name)
        self.add_diff_drive_joint(name=self.drive_joint_name,
                                               parent_link_name=self.odom_link_name,
                                               child_link_name=root_link_name,
                                               translation_limits={
                                                   Derivatives.velocity: 0.2,
                                                   Derivatives.acceleration: 1,
                                                   Derivatives.jerk: 5,
                                               },
                                               rotation_limits={
                                                   Derivatives.velocity: 0.2,
                                                   Derivatives.acceleration: 1,
                                                   Derivatives.jerk: 5
                                               },
                                               robot_group_name=self.robot_group_name)
