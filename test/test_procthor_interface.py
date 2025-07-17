import unittest
import os

from pycram.external_interfaces.procthor import ProcTHORInterface


class TestProcThorInterface(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        cls.procThorInterface = ProcTHORInterface()

    def test_download_tmp(self):
        resource_path, sampled_world = self.procThorInterface.sample_environment()
        self.assertTrue(any(environment in sampled_world for environment in os.listdir(resource_path)))

    def test_download_keep_environment(self):
        resource_path, sampled_world = self.procThorInterface.sample_environment(keep_environment=True)
        self.assertTrue(any(environment in sampled_world for environment in os.listdir(resource_path)))


if __name__ == '__main__':
    unittest.main()
