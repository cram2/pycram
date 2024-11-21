import unittest
import pycrap
import inspect



def recursive_subclasses(cls):
    """
    :param cls: The class.
    :return: A list of the classes subclasses.
    """
    return cls.__subclasses__() + [g for s in cls.__subclasses__() for g in recursive_subclasses(s)]


class CraxTestCase(unittest.TestCase):

    def test_creation(self):
        for cls in recursive_subclasses(pycrap.Base):
            cls: pycrap.Base
            cls.set_comment_to_docstring()
        pycrap.ontology.save()
        self.assertTrue(len(pycrap.PhysicalObject.comment) > 0)

if __name__ == '__main__':
    unittest.main()