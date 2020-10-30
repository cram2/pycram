from copy import deepcopy


class GripperDescription:

    def __init__(self, gripper_links: list[str] = [], gripper_joints: list[str] = [],
                 static_gripper_joint_states: dict[str: list[float]] = {},
                 gripper_meter_to_jnt_multiplier=1.0, gripper_minimal_position=0.0,
                 gripper_convergence_delta=0.001):
        self.links = gripper_links
        self.joints = gripper_joints
        self.static_joint_states = static_gripper_joint_states
        # static params
        self.gripper_meter_to_jnt_multiplier = gripper_meter_to_jnt_multiplier
        self.gripper_minimal_position = gripper_minimal_position
        self.gripper_convergence_delta = gripper_convergence_delta


class ChainDescription:
    """
    This class saves a kinematic chain by saving the links and the joints.
    Moreover, this class offers to set a base_link and specify static joint states
    (e. g. the "park" state of the right arm).

        chain of links: link1 <-> joint1 <-> link2 <-> joint2 <-> ... <-> link6
    """

    def __init__(self, name: str, joints: list[str], links: list[str],
                 base_link=None, static_joint_states: dict[str: list[float]] = {}):
        self.name = name
        self.base_link = base_link
        self.joints = joints
        self.links = links
        self.static_joint_states = static_joint_states

    def add_static_joint_chain(self, configuration: str,
                               static_joint_states: list[float]):
        if len(static_joint_states) == len(self.joints):
            configurations = list(self.static_joint_states.keys())
            if configuration not in configurations:
                self.static_joint_states[configuration] = static_joint_states
            else:
                print("The chain " + self.name + " already has static joint values "
                                                 "for the config " + configuration + ".")
        else:
            print("The number of the joint values does not equal the amount of the joints.")

    def add_static_joint_chains(self, static_joint_states: dict[str: list[float]]):
        for config, joint_states in static_joint_states:
            self.add_static_joint_chain(config, joint_states)


class InteractionDescription(ChainDescription):
    """
    This class allows to put on the end of an chain another link, which is saved
    as an end effector. Therefore, chains can be defined which specify the interaction
    frame for the robot. An example could be a storage place for grasped objects on the robot.

        chain of links: chain_description <-> eef_link
    """

    def __init__(self, chain_description: ChainDescription, eef_link: str):
        tmp = deepcopy(chain_description)
        super().__init__(tmp.name, tmp.joints, tmp.links, tmp.static_joint_states)
        self.chain_description = tmp
        self.eef_link = eef_link

    def add_static_joint_chain(self, configuration: str,
                               static_joint_states: list[float]):
        super().add_static_joint_chain(configuration, static_joint_states)

    def add_static_joint_chains(self, static_joint_states: dict[str: list[float]]):
        super().add_static_joint_chains(static_joint_states)


class ManipulatorDescription(InteractionDescription):
    """
    This class allows with the given interaction description to include a gripper
    description which is placed between the last link of the interaction description
    and the rest of it.
    Independently from that a tool frame can be saved, which allows to use objects
    to manipulate the environment.

        chain of links: interaction_description <-> (gripper_description) <-> (tool_frame)/eef_link
    """

    def __init__(self, interaction_description: InteractionDescription,
                 tool_frame: str = None, gripper_description: GripperDescription = None):
        tmp = deepcopy(interaction_description)
        super().__init__(tmp.chain_description, tmp.eef_link)
        self.interaction_description = tmp
        self.tool_frame = tool_frame
        self.gripper_description = gripper_description


class CameraDescription:

    def __init__(self, frame: str,
                 minimal_height=0.0, maximal_height=0.0,
                 vertical_angle=0.0, horizontal_angle=0.0,
                 other_params: dict[str: float] = {}):
        self.frame = frame
        self.min_height = minimal_height
        self.max_height = maximal_height
        self.vertical_angle = vertical_angle
        self.horizontal_angle = horizontal_angle
        # static, but flexible params
        self.params = other_params


class RobotDescription:

    def __init__(self, odom_frame, base_frame, base_link, torso_link, torso_joint):
        self.interaction_chains = {}  # dict{str: InteractionDescription}
        self.cameras = {}  # dict{str: CameraDescription}
        self.static_transforms = []  # list[tf]
        self.static_poses = []  # list[pose]
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.base_link = base_link
        self.torso_link = torso_link
        self.torso_joint = torso_joint

    def get_static_joint_states(self, arms: list[str], configuration: str):
        pass

    def get_static_tf(self, base_link: str, target_link: str):
        pass

    def get_static_pose(self, frame: str):
        pass

    def add_arm(self, name: str, arm_description: ManipulatorDescription):
        pass

    def add_chain(self, name: str, chain_description: ChainDescription):
        pass

    def add_interaction_chain(self, name: str, interaction_description: InteractionDescription):
        pass

    def add_camera(self, name: str, camera_description: CameraDescription):
        pass

    def add_static_joint_chain(self, chain_name: str, configuration: str,
                               static_joint_states: list[float]):
        self.interaction_chains[chain_name].add_static_joint_chain(configuration, static_joint_states)

    def add_static_joint_chains(self, chain_name: str,
                                static_joint_states: dict[str: list[float]]):
        self.interaction_chains[chain_name].add_static_joint_chains(static_joint_states)

    def add_static_gripper_chain(self, chain_name: str, configuration: str,
                                 static_joint_states: dict[str: list[float]]):
        pass

    def add_static_gripper_chains(self, chain_name: str,
                                  static_joint_states: dict[str: list[float]]):
        pass
