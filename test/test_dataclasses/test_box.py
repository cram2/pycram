import unittest
from pycram.datastructures.dataclasses import BoundingBox

class BoundingBoxTestCase(unittest.TestCase):

    def test_enlarge(self):
        box = BoundingBox(0, 0, 0, 1, 1, 1)
        box.enlarge_all(0.1)
        self.assertEqual(box.min_x, -0.1)
        self.assertEqual(box.max_x, 1.1)

    def test_from_event(self):
        box = BoundingBox(0, 0, 0, 1, 1, 1)
        event = box.simple_event.as_composite_set()
        box2 = BoundingBox.from_event(event)[0]
        self.assertEqual(box, box2)

    def test_contains(self):
        box = BoundingBox(0, 0, 0, 1, 1, 1)
        self.assertTrue(box.contains(0.5, 0.5, 0.5))
        self.assertFalse(box.contains(1.5, 0.5, 0.5))

if __name__ == '__main__':
    unittest.main()
