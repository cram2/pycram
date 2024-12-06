import math
import sys
from time import sleep

from pycram import World
from pycram.robot_description import RobotDescription
from pycram.yarp_utils.yarp_networking import interrupt_and_close
import yarp


class icub_state_updater(yarp.RFModule):

    def __init__(self):
        yarp.RFModule.__init__(self)

        # handle port for the RFModule
        self.handle_port = yarp.Port()
        self.attach(self.handle_port)

        # Define vars to receive an image
        self.module_name = None
        self.robot_name_yarp = None

        self.state_torso_port_name = None
        self.state_right_arm_port_name = None
        self.state_left_arm_port_name = None
        self.state_head_port_name = None

        self.torso_joints_name = None
        self.right_arm_joints_name = None
        self.left_arm_joints_name = None
        self.head_joints_name = None

        self.state_torso_port = yarp.BufferedPortBottle()
        self.state_right_arm_port = yarp.BufferedPortBottle()
        self.state_left_arm_port = yarp.BufferedPortBottle()
        self.state_head_port = yarp.BufferedPortBottle()

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
            print("Can't open the port correctly")
            return False

        if not self.state_torso_port.open(self.state_torso_port_name):
            print(f'Can\'t open the port correctly {self.state_torso_port_name}')
            return False

        if not self.state_right_arm_port.open(self.state_right_arm_port_name):
            print(f'Can\'t open the port correctly {self.state_right_arm_port_name}')
            return False

        if not self.state_left_arm_port.open(self.state_left_arm_port_name):
            print(f'Can\'t open the port correctly {self.state_left_arm_port_name}')
            return False

        if not self.state_head_port.open(self.state_head_port_name):
            print(f'Can\'t open the port correctly {self.state_head_port_name}')
            return False
        if not self.connect_ports():
            print("Error in connecting ports. Make sure that the robot is already up")
            return False


        self.torso_joints_names =  RobotDescription.current_robot_description.get_actuated_joint_names("torso")
        self.right_arm_joints_names = RobotDescription.current_robot_description.get_actuated_joint_names("right_arm")
        self.left_arm_joints_names = RobotDescription.current_robot_description.get_actuated_joint_names("left_arm")
        self.head_joints_names = ["neck_pitch","neck_roll","neck_yaw","eyes_tilt","r_eye_pan_joint","l_eye_pan_joint"]

        if self.torso_joints_names is None:
            print("Error in joint names")
            return False

        if self.right_arm_joints_names is None:
            print("Error in joint names")
            return False

        if self.left_arm_joints_names is None:
            print("Error in joint names")
            return False

        if self.head_joints_names is None:
            print("Error in joint names")
            return False

        print("Initialization complete")
        return True

    def getName(self, name):
        return f'/{self.module_name}{name}'


    def connect_ports(self):
        # status ports
        connection_status = yarp.NetworkBase_connect("/"+self.robot_name_yarp+"/torso/state:o", self.state_torso_port_name, "tcp")
        if not connection_status:
            print("state torso port couldn't connect")
            return False

        connection_status = yarp.NetworkBase_connect("/"+self.robot_name_yarp+"/right_arm/state:o",self.state_right_arm_port_name,  "tcp")
        if not connection_status:
            print("state right_arm port couldn't connect")
            return False

        connection_status = yarp.NetworkBase_connect("/"+self.robot_name_yarp+"/left_arm/state:o",self.state_left_arm_port_name , "tcp")
        if not connection_status:
            print("state left_arm port couldn't connect")
            return False

        connection_status = yarp.NetworkBase_connect("/" + self.robot_name_yarp + "/head/state:o",
                                                     self.state_head_port_name, "tcp")
        if not connection_status:
            print("state head port couldn't connect")
            return False

        return True

    def interruptModule(self):
        print("stopping the module")
        self.handle_port.interrupt()
        self.state_torso_port.interrupt()
        self.state_right_arm_port.interrupt()
        self.state_left_arm_port.interrupt()
        self.state_head_port.interrupt()

        return True

    def close(self):
        self.handle_port.close()
        self.state_torso_port.close()
        self.state_right_arm_port.close()
        self.state_left_arm_port.close()
        self.state_head_port.close()

        return True

    def respond(self, command, reply):

        # Is the command recognized

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
        return 0.5

    def update_part_state(self,part_bottle : yarp.Bottle, part_names: [str]):
        if part_bottle.size()  == len(part_names):
            for i in range(part_bottle.size()):
                try:
                    World.robot.set_joint_position(part_names[i],
                                               math.radians(part_bottle.get(i).asFloat64()))
                except:
                    print("error in updating ",part_names[i])
                    pass

        else :
            print("mismatch in part size" , part_bottle.size() , " ",   len(part_names))

    def update_head_state(self,head_bottle : yarp.Bottle, head_names: [str]):

        for i in range(4):
            try:
                World.robot.set_joint_position(head_names[i],
                                           math.radians(head_bottle.get(i).asFloat64()))
            except:
                print("error in updating head parts ", head_names[i])
                pass


        try:
            r_eye_pan = 0.5 * head_bottle.get(4).asFloat64() - 0.5 * head_bottle.get(5).asFloat64()
            l_eye_pan = 0.5 * head_bottle.get(4).asFloat64() + 0.5 * head_bottle.get(5).asFloat64()

            World.robot.set_joint_position(head_names[4],
                                           math.radians(r_eye_pan))

            World.robot.set_joint_position(head_names[5],
                                           math.radians(l_eye_pan))
        except:
            print("error in updating head parts eyes")
            pass

    def update_arm_state(self,arm_bottle : yarp.Bottle, arms_names: [str],arm:str):
        try:
            print("setting arm joints")
            for i in range(7):
                World.robot.set_joint_position(arms_names[i],
                                           math.radians(arm_bottle.get(i).asFloat64()))
                print(f"set {arms_names[i]} to {math.radians(arm_bottle.get(i).asFloat64())}")
            print("arm joints updated")
            ####################################################################################
            ####################################################################################
            World.robot.set_joint_position(f"{arm}_hand_index_0_joint",
                                           math.radians(arm_bottle.get(7).asFloat64()/-3.0))
            World.robot.set_joint_position(f"{arm}_hand_middle_0_joint",
                                           0)
            World.robot.set_joint_position(f"{arm}_hand_ring_0_joint",
                                           math.radians(20 - (arm_bottle.get(7).asFloat64()/3.0)))
            World.robot.set_joint_position(f"{arm}_hand_little_0_joint",
                                           math.radians(20 - (arm_bottle.get(7).asFloat64()/3.0)))

            ####################################################################################
            # thumb oppose (8) = 0
            print(f"setting thumb joint to {math.radians(arm_bottle.get(8).asFloat64())}")
            World.robot.set_joint_position(f"{arm}_hand_thumb_0_joint",
                                           math.radians(arm_bottle.get(8).asFloat64()))
            print("thumb joint set")

            # # thumb proximal (9) = 1
            World.robot.set_joint_position(f"{arm}_hand_thumb_1_joint",
                                           math.radians(arm_bottle.get(9).asFloat64()))

            # thumb distal (10) = 2 + 3
            World.robot.set_joint_position(f"{arm}_hand_thumb_2_joint",
                                           math.radians(0.5*arm_bottle.get(10).asFloat64()))
            World.robot.set_joint_position(f"{arm}_hand_thumb_3_joint",
                                           math.radians(0.5*arm_bottle.get(10).asFloat64()))
            ####################################################################################
            # # index proximal (11) = 1
            World.robot.set_joint_position(f"{arm}_hand_index_1_joint",
                                           math.radians(arm_bottle.get(11).asFloat64()))

            # index distal (12) = 2 + 3
            World.robot.set_joint_position(f"{arm}_hand_index_2_joint",
                                           math.radians(0.5 * arm_bottle.get(12).asFloat64()))
            World.robot.set_joint_position(f"{arm}_hand_index_3_joint",
                                           math.radians(0.5 * arm_bottle.get(12).asFloat64()))
            ####################################################################################
            # middle proximal (13) = 1
            World.robot.set_joint_position(f"{arm}_hand_middle_1_joint",
                                           math.radians(arm_bottle.get(13).asFloat64()))

            # middle distal (14) = 2 + 3
            World.robot.set_joint_position(f"{arm}_hand_middle_2_joint",
                                           math.radians(0.5 * arm_bottle.get(14).asFloat64()))
            World.robot.set_joint_position(f"{arm}_hand_middle_3_joint",
                                           math.radians(0.5 * arm_bottle.get(14).asFloat64()))
            ####################################################################################

            # Pinky (15) = 1 + 2 + 3 (little) (ring)
            World.robot.set_joint_position(f"{arm}_hand_ring_1_joint",
                                           math.radians(arm_bottle.get(15).asFloat64()/3.0))
            World.robot.set_joint_position(f"{arm}_hand_ring_2_joint",
                                           math.radians(arm_bottle.get(15).asFloat64()/3.0))
            World.robot.set_joint_position(f"{arm}_hand_ring_3_joint",
                                           math.radians( arm_bottle.get(15).asFloat64()/3.0))
            ####################################################################################
            # Pinky (15) = 1 + 2 + 3 (little) (ring)
            World.robot.set_joint_position(f"{arm}_hand_little_1_joint",
                                           math.radians(arm_bottle.get(15).asFloat64()/3.0))
            World.robot.set_joint_position(f"{arm}_hand_little_2_joint",
                                           math.radians(arm_bottle.get(15).asFloat64()/3.0))
            World.robot.set_joint_position(f"{arm}_hand_little_3_joint",
                                           math.radians(arm_bottle.get(15).asFloat64()/3.0))
            ####################################################################################
        except Exception as e:
            print(e)
            print("error in updating arm parts" , f"{arm}_hand_thumb_0_joint")
            pass




    def updateModule(self):
        torso_state: yarp.Bottle = self.state_torso_port.read(shouldWait=False)
        right_arm_state: yarp.Bottle = self.state_right_arm_port.read(shouldWait=False)
        left_arm_state: yarp.Bottle = self.state_left_arm_port.read(shouldWait=False)
        head_state: yarp.Bottle = self.state_head_port.read(shouldWait=False)

        if torso_state is not None:
            self.update_part_state(torso_state,self.torso_joints_names)

        if right_arm_state is not None:
            self.update_arm_state(right_arm_state,self.right_arm_joints_names,"r")

        if left_arm_state is not None:
            self.update_arm_state(left_arm_state,self.left_arm_joints_names,"l")

        if head_state is not None:
            self.update_head_state(head_state,self.head_joints_names)

        return True





def run_icub_state_updater(args):
    if not yarp.Network.checkNetwork():
        print("error in network check")
        return

    m_module = icub_state_updater()

    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext('pycram')
    #rf.setDefaultConfigFile('icub_joint_ipdater.ini')

    if rf.configure(args):
        print("done configuration")
        if m_module.runModuleThreaded(rf) == 0:
            print("done running module")
            return m_module
        else:
            interrupt_and_close(m_module)
            print("Module Failed to open")
            return None


