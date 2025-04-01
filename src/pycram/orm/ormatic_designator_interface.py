from sqlalchemy import Column, DateTime, Enum, Float, ForeignKey, Integer, MetaData, String, Table
from sqlalchemy.orm import registry, relationship
import pycram.orm.logging_hooks

metadata = MetaData()


t_PositionSQL = Table(
    'PositionSQL', metadata,
    Column('id', Integer, primary_key=True),
    Column('x', Float, nullable=False),
    Column('y', Float, nullable=False),
    Column('z', Float, nullable=False)
)

t_QuaternionSQL = Table(
    'QuaternionSQL', metadata,
    Column('id', Integer, primary_key=True),
    Column('x', Float, nullable=False),
    Column('y', Float, nullable=False),
    Column('z', Float, nullable=False),
    Column('w', Float, nullable=False)
)

t_PoseSQL = Table(
    'PoseSQL', metadata,
    Column('id', Integer, primary_key=True),
    Column('position_id', ForeignKey('PositionSQL.id')),
    Column('orientation_id', ForeignKey('QuaternionSQL.id')),
    Column('frame', String, nullable=False),
    Column('header', DateTime, nullable=False)
)

t_MoveTorsoActionSQL = Table(
    'MoveTorsoActionSQL', metadata,
    Column('id', Integer, primary_key=True),
    Column('robot_position_id', ForeignKey('PoseSQL.id')),
    Column('robot_torso_height', Float, nullable=False),
    Column('torso_state', Enum(pycram.datastructures.enums.TorsoState), nullable=False)
)

t_ParkArmsActionSQL = Table(
    'ParkArmsActionSQL', metadata,
    Column('id', Integer, primary_key=True),
    Column('robot_position_id', ForeignKey('PoseSQL.id')),
    Column('robot_torso_height', Float, nullable=False),
    Column('arm', Enum(pycram.datastructures.enums.Arms), nullable=False)
)

mapper_registry = registry(metadata=metadata)

m_ParkArmsActionSQL = mapper_registry.map_imperatively(pycram.orm.logging_hooks.ParkArmsActionSQL, t_ParkArmsActionSQL, properties = dict(robot_position=relationship("PoseSQL")))

m_MoveTorsoActionSQL = mapper_registry.map_imperatively(pycram.orm.logging_hooks.MoveTorsoActionSQL, t_MoveTorsoActionSQL, properties = dict(robot_position=relationship("PoseSQL")))

m_PoseSQL = mapper_registry.map_imperatively(pycram.orm.logging_hooks.PoseSQL, t_PoseSQL, properties = dict(position=relationship("PositionSQL"), 
orientation=relationship("QuaternionSQL")))

m_PositionSQL = mapper_registry.map_imperatively(pycram.orm.logging_hooks.PositionSQL, t_PositionSQL, )

m_QuaternionSQL = mapper_registry.map_imperatively(pycram.orm.logging_hooks.QuaternionSQL, t_QuaternionSQL, )
