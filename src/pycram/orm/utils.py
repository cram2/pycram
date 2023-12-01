import traceback
from anytree import Node, RenderTree, LevelOrderIter
import rospy
import sqlalchemy
import pycram.orm.base
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
import json


def write_database_to_file(in_sessionmaker: sqlalchemy.orm.sessionmaker, filename: str,
                           b_write_to_console: bool = False):
    """
    Writes all Tables stored within the given session into a local file. File will be written in JSON Format

    :param in_sessionmaker: sessionmaker that allows us to access the Database
    :param filename: Filename of the logfile
    :param b_write_to_console: enables writing to the console. Default false
    """
    with in_sessionmaker() as session:
        with open("whatever.txt", "w") as f:
            to_json_dict = dict()
            for table in pycram.orm.base.Base.metadata.sorted_tables:
                list_of_row = list()
                for column_object in session.query(table).all():
                    list_of_row.append(column_object)
                to_json_dict[table.name] = list_of_row
            json_data_dict = json.dumps(to_json_dict, default=str)
            f.write(json_data_dict)


def print_database(in_sessionmaker: sqlalchemy.orm.sessionmaker):
    """
    Prints all ORM Class data within the given Session.

    :param in_sessionmaker: Database Session which should be printed
    """
    with in_sessionmaker() as session:
        for table in pycram.orm.base.Base.metadata.sorted_tables:
            try:
                smt = sqlalchemy.select('*').select_from(table)
                result = session.execute(smt).all()
                rospy.loginfo("Table: {}\tcontent:{}".format(table,result))
            except sqlalchemy.exc.ArgumentError as e:
                print(e)


def get_tree(in_node, parent=None, tree=None):
    if parent is None:
        node = Node(in_node)
        tree = node
    else:
        node = Node(in_node, parent)
    if len(in_node.__subclasses__()):
        for subnode in in_node.__subclasses__():
            get_tree(subnode, node, tree)
    return tree


def update_primary_key(source_session_maker: sqlalchemy.orm.sessionmaker,
                       destination_session_maker: sqlalchemy.orm.sessionmaker):  # source_engine: sqlalchemy.engine.Engine, destination_engine: sqlalchemy.engine.Engine
    """
    Updates all the primary keys of the database associated with the destination engine, so that there will be no
    problems when merging it into the source database. In order to achieve this the highest id value of the source
    engine is searched and the primary keys of the destination database will get all the values following that.
    Cascading triggers in the database will take care of the rest. Careful 2023 this will not work in
    memory databases as there are no triggers.

    :param source_engine: Engine of the source data_base
    :param destination_engine: Engine of the destination data_base
    """
    destination_session = destination_session_maker()  # sqlalchemy.orm.Session(bind=destination_engine)
    source_session = source_session_maker()  # sqlalchemy.orm.Session(bind=source_engine)
    sortedTables = pycram.orm.base.Base.metadata.sorted_tables
    for table in sortedTables:
        try:
            list_of_primary_keys_of_this_table = table.primary_key.columns.values()
            for key in list_of_primary_keys_of_this_table:
                # make it smart but maybe
                all_source_key_values = []
                all_destination_key_values = []
                # print("table:{}\tprimarykeys:{}\tIdInQuestion:{}\tsourcekeyvalues:{} count:{}".format(table,list_of_primary_keys_of_this_table,key,source_session.query(key).all(),source_session.query(key).count()))
                # print("table:{}\tprimarykeys:{}\tIdInQuestion:{}\tdestinationvalues:{} count{}".format(table,list_of_primary_keys_of_this_table,key, destination_session.query(key).all(),destination_session.query(key).count()))
                for key_value_row in source_session.query(key).all():
                    all_source_key_values.append(key_value_row[0])  # get all values of key from source session
                for key_value_row in destination_session.query(key).all():
                    all_destination_key_values.append(key_value_row[0])  # get all values of key from source session

                highest_free_key_value = max(max(all_source_key_values, default=0),
                                             max(all_destination_key_values, default=0)) + 1
                results = destination_session.execute(sqlalchemy.select(table))
                for column_object in results:  # iterate over all columns
                    if column_object.__getattr__(key.name) in all_source_key_values:
                        print("Found primary_key collision in table {} value: {} max value in memory {}".format(table,
                                                                                                                column_object.__getattr__(
                                                                                                                    key.name),
                                                                                                                highest_free_key_value))
                        rospy.loginfo(
                            "Found primary_key collision in table {} value: {} max value in memory {}".format(table,
                                                                                                              column_object.__getattr__(
                                                                                                                  key.name),
                                                                                                              highest_free_key_value))
                        mini_dict = {}
                        mini_dict[key.name] = highest_free_key_value
                        update_statment = sqlalchemy.update(table).where(
                            table.c.id == column_object.__getattr__(key)).values(mini_dict)
                        destination_session.execute(update_statment)
                        highest_free_key_value += 1
            destination_session.commit()  # commit after every table
        except AttributeError as e:
            print(e)
            print("Possible found abstract ORM class {}".format(e.__name__))
    destination_session.close()


def copy_database(source_session_maker: sqlalchemy.orm.sessionmaker,
                  destination_session_maker: sqlalchemy.orm.sessionmaker):
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
    destination_session = destination_session_maker()
    try:
        sortedTables = pycram.orm.base.Base.metadata.sorted_tables
        for table in sortedTables:
            for value in source_session.query(table).all():
                insert_statment = sqlalchemy.insert(table).values(value)
                destination_session.execute(insert_statment)
            destination_session.commit()  # commit after every table
    except Exception as e:
        traceback.print_exc()
    finally:
        source_session.close()
        destination_session.close()


def update_primary_key_constrains(session_maker: sqlalchemy.orm.sessionmaker):
    '''
    Iterates through all tables related to any ORM Class and sets in their corresponding foreign keys in the given
    endpoint to "ON UPDATE CASCADING". Careful currently only works on postgres databases.
    :param session_maker:
    :return: empty
    '''
    with session_maker() as session:
        for table in pycram.orm.base.Base.metadata.sorted_tables:
            try:
                foreign_key_statement = sqlalchemy.text(
                    "SELECT con.oid, con.conname, con.contype, con.confupdtype, con.confdeltype, con.confmatchtype, pg_get_constraintdef(con.oid) FROM pg_catalog.pg_constraint con INNER JOIN pg_catalog.pg_class rel ON rel.oid = con.conrelid INNER JOIN pg_catalog.pg_namespace nsp ON nsp.oid = connamespace WHERE rel.relname = '{}';".format(
                        table))
                response = session.execute(foreign_key_statement)
                print(25 * '~' + "{}".format(table) + 25 * '~')
                for line in response:
                    if line.conname.endswith("fkey"):
                        if 'a' in line.confupdtype:  # a --> no action | if there is no action we set it to cascading
                            # I just assume there aren't any other constraints
                            drop_statement = sqlalchemy.text(
                                "alter table \"{}\" drop constraint \"{}\";".format(table,
                                                                                    line.conname))
                            drop_response = session.execute(
                                drop_statement)  # There is no real data coming back for this
                            alter_statement = sqlalchemy.text(
                                "alter table \"{}\" add constraint {} {} on update cascade;".format(
                                    table,
                                    line.conname,
                                    line.pg_get_constraintdef))
                            alter_response = session.execute(
                                alter_statement)  # There is no real data coming back for this
                            session.commit()
            except AttributeError:
                print("Attribute Error: {} has no attribute __tablename__".format(table))
