import atexit
import time
import threading
from geometry_msgs.msg import WrenchStamped
from semantic_world.world import World
from std_msgs.msg import Header

from ..datastructures.enums import FilterConfig
from ..failures import SensorMonitoringCondition
from ..filter import Butterworth
from ..ros import  Time
from ..ros import  create_publisher, logdebug, loginfo_once, logerr, create_subscriber


class ForceTorqueSensorSimulated:
    """
    Simulated force-torque sensor for a joint with a given name.
    Reads simulated forces and torques at that joint from world and publishes geometry_msgs/Wrench messages
    to the given topic.
    """

    def __init__(self, joint_name, world: World, fts_topic="/pycram/fts", interval=0.1):
        """
        The given joint_name has to be part of :py:attr:`~pycram.world.World.robot` otherwise a
        RuntimeError will be raised.

        :param joint_name: Name of the joint for which force-torque should be simulated
        :param world: The world from which the force-torque values should be read
        :param fts_topic: Name of the ROS topic to which should be published
        :param interval: Interval at which the messages should be published, in seconds
        """
        self.world = world
        self.fts_joint_idx = None
        self.joint_name = joint_name
        # if joint_name in self.world.robot.joint_name_to_id.keys():
        #     self.fts_joint_idx = self.world.robot.joint_name_to_id[joint_name]
        # else:
        #     raise RuntimeError(f"Could not register ForceTorqueSensor: Joint {joint_name}"
        #                        f" does not exist in robot object")
        # self.world.enable_joint_force_torque_sensor(self.world.robot, self.fts_joint_idx)

        self.fts_pub = create_publisher(fts_topic, WrenchStamped, queue_size=10)
        self.interval = interval
        self.kill_event = threading.Event()

        self.thread = threading.Thread(target=self._publish)
        self.thread.start()

        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Continuously publishes the force-torque values for the simulated joint. Values are published as long as the
        kill_event is not set.
        """
        seq = 0
        while not self.kill_event.is_set():
            joint_ft = self.world.get_joint_reaction_force_torque(self.world.robot, self.fts_joint_idx)
            h = Header()
            h.stamp = Time().now()
            h.frame_id = self.joint_name

            wrench_msg = WrenchStamped()
            wrench_msg.header = h
            wrench_msg.wrench.force.x = joint_ft[0]
            wrench_msg.wrench.force.y = joint_ft[1]
            wrench_msg.wrench.force.z = joint_ft[2]

            wrench_msg.wrench.torque.x = joint_ft[3]
            wrench_msg.wrench.torque.y = joint_ft[4]
            wrench_msg.wrench.torque.z = joint_ft[5]

            self.fts_pub.publish(wrench_msg)
            seq += 1
            time.sleep(self.interval)

    def _stop_publishing(self) -> None:
        """
        Sets the kill_event and therefore terminates the Thread publishing the force-torque values as well as join the
        threads.
        """
        self.kill_event.set()
        self.thread.join()


class ForceTorqueSensor:
    """
    Monitor a force-torque sensor of a supported robot and save relevant data.

    Apply a specified filter and save this data as well.
    Default filter is the low pass filter 'Butterworth'

    Can also calculate the derivative of (un-)filtered data

    :param robot_name: Name of the robot
    :param filter_config: Desired filter (default: Butterworth)
    :param filter_order: Order of the filter. Declares the number of elements that delay the sampling
    :param custom_topic: Declare a custom topic if the default topics do not fit
    """

    filtered = 'filtered'
    unfiltered = 'unfiltered'

    def __init__(self, robot_name, filter_config=FilterConfig.butterworth, filter_order=4, custom_topic=None,
                 debug=False):
        self.robot_name = robot_name
        self.filter_config = filter_config
        self.filter = self._get_filter(order=filter_order)
        self.debug = debug

        self.wrench_topic_name = custom_topic
        self.force_torque_subscriber = None
        self.init_data = True

        self.whole_data = None
        self.prev_values = None

        self.order = filter_order

        self._setup()

    def _setup(self):
        self._get_robot_parameters()
        self.subscribe()

    def _get_robot_parameters(self):
        if self.wrench_topic_name is not None:
            return

        if self.robot_name == 'hsrb':
            self.wrench_topic_name = '/hsrb/wrist_wrench/compensated'

        elif self.robot_name == 'iai_donbot':
            self.wrench_topic_name = '/kms40_driver/wrench'

        elif self.robot_name == 'pr2':
            raw_data = "/ft/l_gripper_motor_zeroed"
            raw_data_10_sample_moving_average = "/ft/l_gripper_motor_zeroed_avg"
            derivative_data = "/ft/l_gripper_motor_zeroed_derivative"
            derivative_data_10_sample_moving_average = "/ft/l_gripper_motor_zeroed_derivative_avg"

            self.wrench_topic_name = raw_data
        else:
            logerr(f'{self.robot_name} is not supported')

    def _get_rospy_data(self, data_compensated: WrenchStamped):
        if self.init_data:
            self.init_data = False
            self.prev_values = [data_compensated] * (self.order + 1)
            self.whole_data = {self.unfiltered: [data_compensated],
                               self.filtered: [data_compensated]}

        filtered_data = self._filter_data(data_compensated)

        self.whole_data[self.unfiltered].append(data_compensated)
        self.whole_data[self.filtered].append(filtered_data)

        self.prev_values.append(data_compensated)
        self.prev_values.pop(0)

        if self.debug:
            logdebug(
                f'x: {data_compensated.wrench.force.x}, '
                f'y: {data_compensated.wrench.force.y}, '
                f'z: {data_compensated.wrench.force.z}')

    def _get_filter(self, order=4, cutoff=10, fs=60):
        if self.filter_config == FilterConfig.butterworth:
            return Butterworth(order=order, cutoff=cutoff, fs=fs)

    def _filter_data(self, current_wrench_data: WrenchStamped) -> WrenchStamped:
        filtered_data = WrenchStamped()
        filtered_data.header = current_wrench_data.header
        for attr in ['x', 'y', 'z']:
            force_values = [getattr(val.wrench.force, attr) for val in self.prev_values] + [
                getattr(current_wrench_data.wrench.force, attr)]
            torque_values = [getattr(val.wrench.torque, attr) for val in self.prev_values] + [
                getattr(current_wrench_data.wrench.torque, attr)]

            filtered_force = self.filter.filter(force_values)[-1]
            filtered_torque = self.filter.filter(torque_values)[-1]

            setattr(filtered_data.wrench.force, attr, filtered_force)
            setattr(filtered_data.wrench.torque, attr, filtered_torque)

        return filtered_data

    def subscribe(self):
        """
        Subscribe to the specified wrench topic.

        This will automatically be called on setup.
        Only use this if you already unsubscribed before.
        """
        self.force_torque_subscriber = create_subscriber(self.wrench_topic_name,
                                                            WrenchStamped,
                                                            self._get_rospy_data)

    def unsubscribe(self):
        """
        Unsubscribe from the specified topic
        """
        self.force_torque_subscriber.unregister()

    def get_last_value(self, is_filtered=True) -> WrenchStamped:
        """
        Get the most current data values.

        :param is_filtered: Decides about using filtered or raw data

        :return: A list containing the most current values (newest are first)
        """
        status = self.filtered if is_filtered else self.unfiltered
        return self.whole_data[status][-1]

    def get_derivative(self, is_filtered=True) -> WrenchStamped:
        """
        Calculate the derivative of current data.

        :param is_filtered: Decides about using filtered or raw data.
        :return: The derivative as a WrenchStamped object. Returns a zeroed derivative if only one data point exists.
        """
        status = self.filtered if is_filtered else self.unfiltered

        if len(self.whole_data[status]) < 2:
            return WrenchStamped()

        before: WrenchStamped = self.whole_data[status][-2]
        after: WrenchStamped = self.whole_data[status][-1]
        derivative = WrenchStamped()

        dt = (after.header.stamp - before.header.stamp).to_sec()
        if dt == 0:
            return WrenchStamped()

        derivative.wrench.force.x = float((after.wrench.force.x - before.wrench.force.x) / dt)
        derivative.wrench.force.y = float((after.wrench.force.y - before.wrench.force.y) / dt)
        derivative.wrench.force.z = float((after.wrench.force.z - before.wrench.force.z) / dt)
        derivative.wrench.torque.x = float((after.wrench.torque.x - before.wrench.torque.x) / dt)
        derivative.wrench.torque.y = float((after.wrench.torque.y - before.wrench.torque.y) / dt)
        derivative.wrench.torque.z = float((after.wrench.torque.z - before.wrench.torque.z) / dt)

        return derivative

    def human_touch_monitoring(self, plan):
        while True:
            loginfo_once("Now monitoring for human touch")
            if self.robot_name == 'pr2':
                der = self.get_derivative()
                if abs(der.wrench.torque.x) > 4:
                    plan.root.resume()
                    break
        return False


