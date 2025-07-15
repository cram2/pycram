import unittest
import os
from unittest.mock import patch, MagicMock
from pycram.datastructures.enums import FilterConfig
try:
    from geometry_msgs.msg import WrenchStamped
    from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor, ForceTorqueSensorSimulated
except ImportError:
    pass

@unittest.skipIf("ROS_VERSION" not in os.environ, "ROS is not available")
class ForceTorqueSensorTestCase(unittest.TestCase):

    @patch('pycram.ros_utils.force_torque_sensor.World')
    @patch('pycram.ros_utils.force_torque_sensor.create_publisher')
    def test_initialization_simulated_sensor(self, mock_create_publisher, mock_world):
        mock_world.current_world.robot.joint_name_to_id = {'joint1': 1}
        sensor = ForceTorqueSensorSimulated('joint1')
        self.assertEqual('joint1', 'joint1')
        self.assertIsNotNone(sensor.fts_pub)
        sensor._stop_publishing()

    @patch('pycram.ros_utils.force_torque_sensor.World')
    @patch('pycram.ros_utils.force_torque_sensor.create_publisher')
    def test_initialization_simulated_sensor_invalid_joint(self, mock_create_publisher, mock_world):
        mock_world.current_world.robot.joint_name_to_id = {}
        with self.assertRaises(RuntimeError):
            ForceTorqueSensorSimulated('invalid_joint')

    def test_initialization_force_torque_sensor(self):
        sensor = ForceTorqueSensor('hsrb')
        self.assertEqual(sensor.robot_name, 'hsrb')
        self.assertEqual(sensor.wrench_topic_name, '/hsrb/wrist_wrench/compensated')

    def test_get_last_value(self):
        sensor = ForceTorqueSensor('hsrb')
        mock_data = MagicMock(spec=WrenchStamped)
        sensor.whole_data = {sensor.filtered: [mock_data], sensor.unfiltered: [mock_data]}
        self.assertEqual(sensor.get_last_value(), mock_data)

    def test_get_derivative(self):
        sensor = ForceTorqueSensor('hsrb')
        mock_data1 = MagicMock(spec=WrenchStamped)
        mock_data2 = MagicMock(spec=WrenchStamped)
        sensor.whole_data = {sensor.filtered: [mock_data1, mock_data2], sensor.unfiltered: [mock_data1, mock_data2]}
        derivative = sensor.get_derivative()
        self.assertIsInstance(derivative, WrenchStamped)

if __name__ == '__main__':
    unittest.main()