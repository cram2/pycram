from unittest import TestCase, skipIf
try:
    from pycram.object_descriptors.mjcf import ObjectDescription as MJCFObjDesc
    from dm_control import mjcf
except ImportError:
    MJCFObjDesc = None


@skipIf(MJCFObjDesc is None, "Multiverse not found.")
class TestMjcf(TestCase):
    model: MJCFObjDesc

    @classmethod
    def setUpClass(cls):
        # Example usage
        model = mjcf.RootElement("test")

        model.default.dclass = 'default'

        # Define a simple model with bodies and joints
        body1 = model.worldbody.add('body', name='body1')
        body2 = body1.add('body', name='body2')
        joint1 = body2.add('joint', name='joint1', type='hinge')

        body3 = body2.add('body', name='body3')
        joint2 = body3.add('joint', name='joint2', type='slide')

        cls.model = MJCFObjDesc()
        cls.model.update_description_from_string(model.to_xml_string())

    def test_child_map(self):
        self.assertEqual(self.model.child_map, {'body1': [('joint1', 'body2')], 'body2': [('joint2', 'body3')]})

    def test_parent_map(self):
        self.assertEqual(self.model.parent_map, {'body2': ('joint1', 'body1'), 'body3': ('joint2', 'body2')})

    def test_get_chain(self):
        self.assertEqual(self.model.get_chain('body1', 'body3'),
                         ['body1', 'joint1', 'body2', 'joint2', 'body3'])

