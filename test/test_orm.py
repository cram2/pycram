import unittest
import pycram.orm.base
import pycram.orm.task
import pycram.task
from pycram.task import with_tree
import test_task_tree
import anytree
import sqlalchemy
import sqlalchemy.orm
import os


class ORMTestCase(unittest.TestCase):

    def test_base(self):
        instance = pycram.orm.base.Base()
        self.assertEqual("Base", instance.__tablename__)


class ORMTaskTreeTestCase(test_task_tree.TaskTreeTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=True)

    def setUp(self):
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session = sqlalchemy.orm.Session(bind=self.engine)

    def tearDown(self):
        self.session.close()

    def test_node(self):
        """Test if the number of rows in the node table is equal to the task tree size"""
        self.plan()
        pycram.task.task_tree.root.insert(self.session)
        node_results = self.session.query(pycram.orm.task.TaskTreeNode).all()
        code_results = self.session.query(pycram.orm.task.Code).all()
        self.assertEqual(len(node_results), len(pycram.task.task_tree.root))
        print(*code_results)
        self.assertEqual(len(code_results), len(pycram.task.task_tree.root))


    @classmethod
    def TearDownClass(cls):
        pycram.orm.base.Base.metadata.drop_all(cls.engine)
        cls.session.commit()




if __name__ == '__main__':
    unittest.main()
