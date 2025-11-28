import unittest
from pycram.filter import Butterworth


class TestButterworthFilter(unittest.TestCase):

    def test_initialization_with_default_values(self):
        filter = Butterworth()
        self.assertEqual(filter.order, 4)
        self.assertEqual(filter.cutoff, 10)
        self.assertEqual(filter.fs, 60)

    def test_initialization_with_custom_values(self):
        filter = Butterworth(order=2, cutoff=5, fs=30)
        self.assertEqual(filter.order, 2)
        self.assertEqual(filter.cutoff, 5)
        self.assertEqual(filter.fs, 30)

    def test_filter_data_with_default_values(self):
        filter = Butterworth()
        data = [1, 2, 3, 4, 5]
        filtered_data = filter.filter(data)
        self.assertEqual(len(filtered_data), len(data))

    def test_filter_data_with_custom_values(self):
        filter = Butterworth(order=2, cutoff=5, fs=30)
        data = [1, 2, 3, 4, 5]
        filtered_data = filter.filter(data)
        self.assertEqual(len(filtered_data), len(data))

    def test_filter_empty_data(self):
        filter = Butterworth()
        data = []
        filtered_data = filter.filter(data)
        self.assertEqual(filtered_data.tolist(), data)

    def test_filter_single_value_data(self):
        filter = Butterworth()
        data = [1]
        filtered_data = filter.filter(data)
        expected_filtered_data = 0.026077721701092293  # The expected filtered value
        # self.assertAlmostEquals(filtered_data.tolist()[0], expected_filtered_data)
        self.assertAlmostEqual(filtered_data.tolist()[0], expected_filtered_data)


if __name__ == "__main__":
    unittest.main()
