import math

from pycram import World
from pycram.failures import YarpNetworkError
from pycram.robot_description import RobotDescription
from pycram.ros.logging import logdebug, logwarn, loginfo, logerr
from pycram.external_interfaces.yarp_networking import *


class IcubStateUpdater(yarp.RFModule):

    def __init__(self):
        yarp.RFModule.__init__(self)


        self.handle_port = yarp.Port()
        self.attach(self.handle_port)


        self.module_name = None
        self.robot_name_yarp = None

        self.state_torso_port_name = None
        self.state_right_arm_port_name = None
        self.state_left_arm_port_name = None
        self.state_head_port_name = None

        self.state_torso_port = yarp.BufferedPortBottle()
        self.state_right_arm_port = yarp.BufferedPortBottle()
        self.state_left_arm_port = yarp.BufferedPortBottle()
        self.state_head_port = yarp.BufferedPortBottle()

        self.torso_joints_names = None
        self.right_arm_joints_names = None
        self.left_arm_joints_names = None
        self.head_joints_names = None

    def configure(self, rf):
        self.module_name = rf.check("name",
                                    yarp.Value("pycramICubStatePublisher"),
                                    "module name (string)").asString()

        self.robot_name_yarp = rf.check("robot",
                                    yarp.Value("icub"),
                                    "module name (string)").asString()

        self.state_torso_port_name = self.getName("/state/torso:i")
        self.state_right_arm_port_name = self.getName("/state/right_arm:i")
        self.state_left_arm_port_name = self.getName("/state/left_arm:i")
        self.state_head_port_name = self.getName("/state/head:i")


        # Create handle port to read message
        if not self.handle_port.open('/' + self.module_name):
            logerr("Can't open the port correctly")
            return False

        if not self.state_torso_port.open(self.state_torso_port_name):
            logerr(f'Can\'t open the port correctly {self.state_torso_port_name}')
            return False

        if not self.state_right_arm_port.open(self.state_right_arm_port_name):
            logerr(f'Can\'t open the port correctly {self.state_right_arm_port_name}')
            return False

        if not self.state_left_arm_port.open(self.state_left_arm_port_name):
            logerr(f'Can\'t open the port correctly {self.state_left_arm_port_name}')
            return False

        if not self.state_head_port.open(self.state_head_port_name):
            logerr(f'Can\'t open the port correctly {self.state_head_port_name}')
            return False
        if not self.connect_ports():
            logerr("Error in connecting ports. Make sure that the robot is already up")
            return False


        self.torso_joints_names =  RobotDescription.current_robot_description.get_actuated_joint_names("torso")
        self.right_arm_joints_names = RobotDescription.current_robot_description.get_actuated_joint_names("right_arm")
        self.left_arm_joints_names = RobotDescription.current_robot_description.get_actuated_joint_names("left_arm")
        self.head_joints_names = ["neck_pitch","neck_roll","neck_yaw","eyes_tilt","r_eye_pan_joint","l_eye_pan_joint"]

        loginfo("Initialization complete")
        return True

    def getName(self, name):
        """
        Constructs a full YARP-compatible port name for the module.

        This method appends a given string (`name`) to the module's base name
        (`self.module_name`) to construct a complete port name in the YARP
        naming convention. The port name is returned as a formatted string
        prefixed with a forward slash (`/`), which is the standard for YARP ports.

        Args:
            name (str): The specific name to append to the module's base name.

        Returns:
            str: The fully constructed YARP-compatible port name.
        """
        return f'/{self.module_name}{name}'


    def connect_ports(self):
        """
        Establishes connections between the robot's state output ports and the module's state input ports.

        This method uses the YARP (Yet Another Robot Platform) networking library to connect the robot's
        state output ports (torso, right arm, left arm, head) to the corresponding state input ports in
        the module. The connections are established using the TCP protocol to ensure reliable communication.

        If any connection fails, an error message is printed to indicate which port could not connect,
        and the method returns `False`. If all connections are successfully established, the method
        returns `True`.

        Returns:
            bool: `True` if all ports successfully connect, `False` otherwise.
        """
        # Torso state port
        connection_status = yarp.NetworkBase_connect("/"+self.robot_name_yarp+"/torso/state:o", self.state_torso_port_name, "tcp")
        if not connection_status:
            logdebug("state torso port couldn't connect")
            return False

        # Right arm state port
        connection_status = yarp.NetworkBase_connect("/"+self.robot_name_yarp+"/right_arm/state:o",self.state_right_arm_port_name,  "tcp")
        if not connection_status:
            logdebug("state right_arm port couldn't connect")
            return False

        # Left Arm state port
        connection_status = yarp.NetworkBase_connect("/"+self.robot_name_yarp+"/left_arm/state:o",self.state_left_arm_port_name , "tcp")
        if not connection_status:
            logdebug("state left_arm port couldn't connect")
            return False

        # Head state port
        connection_status = yarp.NetworkBase_connect("/" + self.robot_name_yarp + "/head/state:o",
                                                     self.state_head_port_name, "tcp")
        if not connection_status:
            logdebug("state head port couldn't connect")
            return False

        return True

    def interruptModule(self):

        """
        Interrupts all communication ports associated with this object.

        This method is used to temporarily halt the operation of the module by
        interrupting the communication ports. Interrupting the ports allows the
        system to pause ongoing communication or processing without permanently
        closing the connections.

        Returns:
            bool: `True` if all ports were successfully interrupted.
        """

        self.handle_port.interrupt()
        self.state_torso_port.interrupt()
        self.state_right_arm_port.interrupt()
        self.state_left_arm_port.interrupt()
        self.state_head_port.interrupt()

        return True

    def close(self):
        """
        Closes all the communication ports associated with this object.

        This method is responsible for ensuring that all the communication ports
        used by the system are properly closed to free up resources and maintain
        system integrity. The ports being closed include:

        - `handle_port`: The port used for handling general rpc commands for the yarp module.
        - `state_torso_port`: The port used for monitoring the torso's state.
        - `state_right_arm_port`: The port used for monitoring the right arm's state.
        - `state_left_arm_port`: The port used for monitoring the left arm's state.
        - `state_head_port`: The port used for monitoring the head's state.

        After closing all the ports, the function returns `True` to indicate
        successful closure of all resources.

        Returns:
            bool: `True` if all ports were successfully closed.
        """

        self.handle_port.close()
        self.state_torso_port.close()
        self.state_right_arm_port.close()
        self.state_left_arm_port.close()
        self.state_head_port.close()

        return True

    def respond(self, command, reply):

        """
        Processes a command and generates an appropriate response.

        This method handles incoming commands (RPC) through the port /module_name
        Further it execute a behaviour and then replies
        """
        reply.clear()

        if command.get(0).asString() == "quit":
            reply.addString("quitting")
            return False
        else:
            reply.addString("other request")

        return True


    def getPeriod(self):
        """
           Module refresh rate.
           Returns : The period of the module in seconds.
        """
        return 1

    def update_joint_degree(self,joint_name:str,joint_value_degree:float):
        try:
            World.robot.set_joint_position(joint_name,math.radians(joint_value_degree))

        except Exception as e:
            logerr(f"error in updating joint {joint_name} to {joint_value_degree} degree")
            pass



    def update_torso_state(self, part_bottle : yarp.Bottle, part_names: [str]):
        if part_bottle.size()  == len(part_names):
            for i in range(part_bottle.size()):
                self.update_joint_degree(part_names[i],
                                           part_bottle.get(i).asFloat64())
        else :
            logwarn(f"mismatch in torso joint sizes {part_bottle.size()} , {len(part_names)}")

    def update_head_state(self,head_bottle : yarp.Bottle, head_names: [str]):
        for i in range(4):
            self.update_joint_degree(head_names[i],
                                       head_bottle.get(i).asFloat64())


        r_eye_pan = 0.5 * head_bottle.get(4).asFloat64() - 0.5 * head_bottle.get(5).asFloat64()
        l_eye_pan = 0.5 * head_bottle.get(4).asFloat64() + 0.5 * head_bottle.get(5).asFloat64()

        self.update_joint_degree(head_names[4],r_eye_pan)
        self.update_joint_degree(head_names[5],l_eye_pan)



    def update_arm_state(self,arm_bottle : yarp.Bottle, arms_names: [str],arm:str):
        for i in range(7):
            self.update_joint_degree(arms_names[i],arm_bottle.get(i).asFloat64())
        ####################################################################################
        ####################################################################################
        # hand (7) = index 0 +  ring 0 + little 0
        self.update_joint_degree(f"{arm}_hand_index_0_joint",
                                       arm_bottle.get(7).asFloat64()/-3.0)
        self.update_joint_degree(f"{arm}_hand_middle_0_joint",
                                       0)
        self.update_joint_degree(f"{arm}_hand_ring_0_joint",
                                       20 - (arm_bottle.get(7).asFloat64()/3.0))
        self.update_joint_degree(f"{arm}_hand_little_0_joint",
                                       20 - (arm_bottle.get(7).asFloat64()/3.0))

        ####################################################################################
        # thumb oppose (8) = 0

        self.update_joint_degree(f"{arm}_hand_thumb_0_joint",
                                       arm_bottle.get(8).asFloat64())

        # # thumb proximal (9) = 1
        self.update_joint_degree(f"{arm}_hand_thumb_1_joint",
                                       arm_bottle.get(9).asFloat64())

        # thumb distal (10) = 2 + 3
        self.update_joint_degree(f"{arm}_hand_thumb_2_joint",
                                       0.5*arm_bottle.get(10).asFloat64())
        self.update_joint_degree(f"{arm}_hand_thumb_3_joint",
                                       0.5*arm_bottle.get(10).asFloat64())
        ####################################################################################
        # # index proximal (11) = 1
        self.update_joint_degree(f"{arm}_hand_index_1_joint",
                                       arm_bottle.get(11).asFloat64())

        # index distal (12) = 2 + 3
        self.update_joint_degree(f"{arm}_hand_index_2_joint",
                                       0.5 * arm_bottle.get(12).asFloat64())
        self.update_joint_degree(f"{arm}_hand_index_3_joint",
                                       0.5 * arm_bottle.get(12).asFloat64())
        ####################################################################################
        # middle proximal (13) = 1
        self.update_joint_degree(f"{arm}_hand_middle_1_joint",
                                       arm_bottle.get(13).asFloat64())

        # middle distal (14) = 2 + 3
        self.update_joint_degree(f"{arm}_hand_middle_2_joint",
                                       0.5 * arm_bottle.get(14).asFloat64())
        self.update_joint_degree(f"{arm}_hand_middle_3_joint",
                                       0.5 * arm_bottle.get(14).asFloat64())
        ####################################################################################

        # Pinky (15) = 1 + 2 + 3 (little) (ring)
        self.update_joint_degree(f"{arm}_hand_ring_1_joint",
                                       arm_bottle.get(15).asFloat64()/3.0)
        self.update_joint_degree(f"{arm}_hand_ring_2_joint",
                                       arm_bottle.get(15).asFloat64()/3.0)
        self.update_joint_degree(f"{arm}_hand_ring_3_joint",
                                        arm_bottle.get(15).asFloat64()/3.0)
        ####################################################################################
        # Pinky (15) = 1 + 2 + 3 (little) (ring)
        self.update_joint_degree(f"{arm}_hand_little_1_joint",
                                       arm_bottle.get(15).asFloat64()/3.0)
        self.update_joint_degree(f"{arm}_hand_little_2_joint",
                                       arm_bottle.get(15).asFloat64()/3.0)
        self.update_joint_degree(f"{arm}_hand_little_3_joint",
                                       arm_bottle.get(15).asFloat64()/3.0)
        ####################################################################################





    def updateModule(self):
        torso_state: yarp.Bottle = self.state_torso_port.read(shouldWait=False)
        right_arm_state: yarp.Bottle = self.state_right_arm_port.read(shouldWait=False)
        left_arm_state: yarp.Bottle = self.state_left_arm_port.read(shouldWait=False)
        head_state: yarp.Bottle = self.state_head_port.read(shouldWait=False)

        if torso_state is not None:
            self.update_torso_state(torso_state, self.torso_joints_names)

        if right_arm_state is not None:
            self.update_arm_state(right_arm_state,self.right_arm_joints_names,"r")

        if left_arm_state is not None:
            self.update_arm_state(left_arm_state,self.left_arm_joints_names,"l")

        if head_state is not None:
            self.update_head_state(head_state,self.head_joints_names)

        return True





def run_icub_state_updater(args):
    if not yarp.Network.checkNetwork():
        raise YarpNetworkError()

    m_module = IcubStateUpdater()

    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext('pycram')
    #rf.setDefaultConfigFile('icub_joint_ipdater.ini')

    if rf.configure(args):
        logdebug("icub_state_updater Configuration done")
        if m_module.runModuleThreaded(rf) == 0:
            loginfo("done running module")
            return m_module
        else:
            interrupt_and_close(m_module)
            logerr("Module Failed to open")
            return None


