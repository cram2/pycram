import traceback

import rospy
import sqlalchemy
import pycram.orm.base
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *


def write_database_to_file(in_session: sqlalchemy.orm.Session, filename: str, b_write_to_console: bool = False):
    """
    Writes all ORM Objects stored within the given session into a local file.

    :param in_session: Database Session which should be logged
    :param filename: Filename of the logfile
    :param b_write_to_console: enables writing to the console. Default false
    """
    with open(filename, "w") as f:
        for table in Base.__subclasses__():
            for column_object in in_session.query(table).all():
                if b_write_to_console:
                    rospy.loginfo("I am writing: {}".format(str(column_object.__dict__)))
                f.write(str(column_object.__dict__))
                f.write("\n")


def print_database(in_Sessionmaker: sqlalchemy.orm.Sessionmaker):
    """
    Prints all ORM Class data within the given Session.

    :param in_sessionmaker: Database Session which should be printed
    """
    memory_session = in_Sessionmaker()
    for table in Base.__subclasses__():
        smt = sqlalchemy.select('*').select_from(table)
        result = memory_session.execute(smt).all()
        rospy.loginfo(result)


def update_primary_key(source_engine: sqlalchemy.orm.Engine, destination_engine: sqlalchemy.orm.Engine):
    """
    Updates all the primary keys of the database associated with the destination engine, so that there will be no
    problems when merging it into the source database. In order to achieve this the highest id value of the source
    engine is searched and the primary keys of the destination database will get all the values following that.
    Cascading triggers in the database will take care of the rest. Careful 2023 this will not work in
    memory databases as there are no triggers.

    :param source_engine: Engine of the source data_base
    :param destination_engine: Engine of the destination data_base
    """
    destination_session = sqlalchemy.orm.Session(bind=destination_engine)
    source_session = sqlalchemy.orm.Session(bind=source_engine)
    primary_keys = {}
    for table in Base.__subclasses__():  # iterate over all tables
        highest_free_key_value = 0
        primary_keys[table] = {}
        list_of_primary_keys_of_this_table = table.__table__.primary_key.columns.values()
        for key in list_of_primary_keys_of_this_table:
            # make it smart but maybe
            all_memory_key_values = source_session.query(key).all()
            primary_keys[table][key.name] = destination_session.query(key).all()
            if all_memory_key_values:
                highest_free_key_value = max(all_memory_key_values)[
                                             0] + 1
            for column_object in destination_session.query(table).all():  # iterate over all columns
                if column_object.__dict__[key.name] in all_memory_key_values:
                    rospy.loginfo(
                        "Found primary_key collision in table {} value: {} max value in memory {}".format(table,
                                                                                                          column_object.__dict__[
                                                                                                              key.name],
                                                                                                          highest_free_key_value))
                    column_object.__dict__[key.name] = highest_free_key_value
                    highest_free_key_value += 1

    destination_session.commit()
    destination_session.close()


def copy_database(source_session_maker: sqlalchemy.orm.Sessionmaker,
                  destination_session_maker: sqlalchemy.orm.Sessionmaker):
    """
    Iterates through all ORM Objects within tht source database and merges them into the destination database. Careful
    this function does not check if there are any primary key collisions or updates any data.

     .. note::
        Ignores all previously detached data, could result in loss of information. During testing database objects
        sometimes had a detached twin. As a possible feature in the future it maybe useful to give the user an
        opportunity to decide what happens with the detached objects. Careful this could lead to duplicated data in the
        destination database.

    :param source_session_maker: Sessionmaker of the source database
    :param destination_session_maker: Sessionmaker of the destination database
    """
    source_session = source_session_maker()
    objects_to_add = []
    try:
        for orm_object_class in Base.__subclasses__():
            result = source_session.execute(
                sqlalchemy.select(orm_object_class).options(sqlalchemy.orm.joinedload('*'))).mappings().all()
            for row in result:
                for key in row:
                    if not sqlalchemy.inspect(row[key]).detached and not sqlalchemy.inspect(
                            row[key]).transient and not sqlalchemy.inspect(row[key]).deleted:
                        source_session.refresh(row[key])
                        source_session.expunge(row[key])
                        sqlalchemy.orm.make_transient(row[key])
                        objects_to_add.append(row[key])
                    else:
                        rospy.logwarn("WARNING: Ignored already detached ORM Object {} ".format(
                            row[key]))
    except Exception as e:
        traceback.print_exc()
    finally:
        source_session.close()
    if len(objects_to_add) < 1:
        return
    destination_session = destination_session_maker()

    destination_session.add_all(objects_to_add)
    destination_session.commit()
    destination_session.close()
