import unittest
import os

from pycram.external_interfaces.procthor import ProcThorInterface
from pycram.ros import ros_tools
import socket
import shutil


def is_server_reachable(host, port):
    """Check if a server is reachable by attempting to open a socket connection."""
    try:
        with socket.create_connection((host, port), timeout=5):
            return True
    except (socket.timeout, OSError):
        return False


class TestProcThorInterface(unittest.TestCase):
    @unittest.skipIf(not(is_server_reachable("procthor.informatik.uni-bremen.de", 5000)),"Host unreachable")
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        cls.test_folder_path = os.path.join(ros_tools.get_ros_package_path('pycram') + "/tmp")
        cls.procThorInterface = ProcThorInterface(base_url="http://procthor.informatik.uni-bremen.de:5000/",
                                                  source_folder=cls.test_folder_path)

    def setUp(self) -> None:
        super().setUp()
        if not os.path.exists(self.test_folder_path):
            os.mkdir(self.test_folder_path)

    def tearDown(self) -> None:
        if os.path.exists(self.test_folder_path):
            shutil.rmtree(self.test_folder_path)

    @classmethod
    def TearDownClass(cls):
        super().TearDownClass()


    def test_download_one(self):
        self.procThorInterface.download_one_random_environment()
        found_dir=[name for name in os.listdir(self.test_folder_path) if os.path.isdir(os.path.join(self.test_folder_path,name))]
        self.assertTrue(1==len(found_dir))

    def test_download_multiple(self):
        self.procThorInterface.download_num_random_environment(3)
        found_dir = [name for name in os.listdir(self.test_folder_path) if
                     os.path.isdir(os.path.join(self.test_folder_path, name))]
        self.assertTrue(3 == len(found_dir))

    def test_get_all_environments_stored_below_dictionary(self):
        before=self.procThorInterface.get_all_environments_stored_below_directory(self.test_folder_path)
        self.procThorInterface.download_one_random_environment()
        after=self.procThorInterface.get_all_environments_stored_below_directory(self.test_folder_path)
        self.assertTrue(1==len(after)-len(before))


if __name__ == '__main__':
    unittest.main()
