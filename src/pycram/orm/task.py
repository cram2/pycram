import sqlalchemy.orm
from .base import Base
import datetime


class TaskTreeNode(Base):

    __tablename__ = "TaskTreeNode"
    id: sqlalchemy.orm.Mapped[int] = sqlalchemy.orm.mapped_column(primary_key=True, autoincrement=True)
    start_time: sqlalchemy.orm.Mapped[datetime.datetime]
