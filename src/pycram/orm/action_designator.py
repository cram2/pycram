from .base import *


class ParkArmsDesignator(Base):

    __tablename__ = "ParkArms"
    id = sqlalchemy.Column(sqlalchemy.types.Integer, autoincrement=True, primary_key=True)
    arm = sqlalchemy.Column(sqlalchemy.types.String, nullable=False)

    def __init__(self, arm: str = None):
        super().__init__()
        self.arm = arm
