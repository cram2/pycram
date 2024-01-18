import test_bullet_world


class TestAttachment(test_bullet_world.BulletWorldTest):

    def test_attach(self):
        self.milk.attach(self.robot)
        self.assertTrue(self.milk.attachments[self.robot])
        self.assertTrue(self.robot.attachments[self.milk])

    def test_detach(self):
        self.milk.attach(self.robot)
        self.milk.detach(self.robot)
        self.assertTrue(self.robot not in self.milk.attachments)
        self.assertTrue(self.milk not in self.robot.attachments)

    def test_attachment_behavior(self):
        self.robot.attach(self.milk)

        milk_pos = self.milk.get_position()
        rob_pos = self.robot.get_position()

        rob_pos.x += 1
        self.robot.set_position(rob_pos)

        new_milk_pos = self.milk.get_position()
        self.assertEqual(new_milk_pos.x, milk_pos.x + 1)

    def test_detachment_behavior(self):
        self.robot.attach(self.milk)

        milk_pos = self.milk.get_position()
        rob_pos = self.robot.get_position()

        self.robot.detach(self.milk)
        rob_pos.x += 1
        self.robot.set_position(rob_pos)

        new_milk_pos = self.milk.get_position()
        self.assertEqual(new_milk_pos.x, milk_pos.x)


