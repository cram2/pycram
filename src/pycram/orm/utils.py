import traceback

import sqlalchemy
import pycram.orm.base
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *


def logDB(in_session, filename, b_write_to_console=False):
    """
        Utility function which stores a database in a local file.

        :param in_session: (sqlalchemy.orm.session) Database Session which should be logged
        :param filename: (String) Filename of the logfile
        :param b_write_to_console: (bool) enables writing to the console. Default false
        """
    f = open(filename, "w")
    for table in Base.__subclasses__():
        for column_object in in_session.query(table).all():
            if b_write_to_console:
                print("I am writing: {}".format(str(column_object.__dict__)))
            f.write(str(column_object.__dict__))
            f.write("\n")
    f.close()


def printDB(in_SessionMaker):
    """
        Prints all ORM Class data within the given Session.

        :param in_session: (sqlalchemy.orm.session) Database Session which should be printed
        """
    memory_session = in_SessionMaker()
    for table in Base.__subclasses__():
        smt = sqlalchemy.select('*').select_from(table)
        result = memory_session.execute(smt).all()
        print(result)


def update_primary_key(source_engine, destination_engine):
    '''
    Primary keys of destination will be updated so that there are no collisions when merging.
    Care it is expected that the database takes care of cascading the change through its foreign keys
    Will create own session

    :param source_engine: (sqlalchemy.orm.engine) Engine of the source data_base
    :param destination_engine: (sqlalchemy.orm.engine) Engine of the destination data_base
    '''
    real_session = sqlalchemy.orm.Session(bind=destination_engine)
    memory_session = sqlalchemy.orm.Session(bind=source_engine)
    primaryKeys = {}
    for table in Base.__subclasses__():  # iterate over all tables
        highest_free_keyValue = 0
        primaryKeys[table] = {}
        list_of_primary_keys_of_this_table = table.__table__.primary_key.columns.values()
        for key in list_of_primary_keys_of_this_table:
            # make it smart but maybe
            all_memory_key_values = memory_session.query(key).all()
            primaryKeys[table][key.name] = real_session.query(key).all()
            if all_memory_key_values:
                highest_free_keyValue = max(all_memory_key_values)[0] + 1  # ToDo: Make it even more generic? # We will add 1 to the id then let the DB to the rest, afterwards we will compy all info over
            for column_object in real_session.query(table).all():  # iterate over all columns
                if column_object.__dict__[key.name] in all_memory_key_values:
                    print("Found primarykey collision in table {} value: {} max value in memory {}".format(table,
                                                                                                           column_object.__dict__[
                                                                                                               key.name],
                                                                                                           highest_free_keyValue))
                    column_object.__dict__[key.name] = highest_free_keyValue
                    highest_free_keyValue += 1

    real_session.commit()
    real_session.close()


def copy_database(source_session_maker, destination_session_maker):
    '''
    Primary keys of destination will be updated so that there are no collisions when merging.
    Care it is expected that the database takes care of cascading the change through its foreign keys
    Will create own session

    :param source_session_maker: (sqlalchemy.orm.sessionmaker) Sessionmaker of the source database
    :param destination_session_maker: (sqlalchemy.orm.sessionmaker) Sessionmaker of the destination database
    '''
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
                        print("WARNING: Ignored already detached ORM Object {} ".format(
                            row[key]))  # ToDo: Maybe give option to include theses items (could break something)
    except Exception as e:
        traceback.print_exc()
        # logging.error(traceback.format_exc())
    finally:
        source_session.close()
    if len(objects_to_add) < 1:
        return
    destination_session = destination_session_maker()

    destination_session.add_all(objects_to_add)
    destination_session.commit()
    destination_session.close()
