import traceback
from anytree import Node,RenderTree,LevelOrderIter
import rospy
import sqlalchemy
import pycram.orm.base
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *



def write_database_to_file(in_session: sqlalchemy.orm.session, filename: str, b_write_to_console: bool = False):
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


def print_database(in_Sessionmaker: sqlalchemy.orm.sessionmaker):
    """
    Prints all ORM Class data within the given Session.

    :param in_sessionmaker: Database Session which should be printed
    """
    memory_session = in_Sessionmaker()
    for table in Base.__subclasses__():
        smt = sqlalchemy.select('*').select_from(table)
        result = memory_session.execute(smt).all()
        rospy.loginfo(result)


def get_all_children_set(in_node, node_set=None):
    all_nodes = set()
    if node_set is None:
        node_set = set()
    node_set.add(in_node)
    if in_node.__subclasses__():
        for node in in_node.__subclasses__():
            all_nodes.update(get_all_children_set(node, node_set))
    else:
        pass
        all_nodes.update(node_set)
    return all_nodes


def get_tree(in_node,parent=None,tree=None):
    if parent is None:
        node=Node(in_node)
        tree=node
    else:
        node=Node(in_node,parent)
    if len(in_node.__subclasses__()):
        for subnode in in_node.__subclasses__():
            get_tree(subnode,node,tree)
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
    primary_keys = {}
    orm_tree=get_tree(pycram.orm.base.Base)
    ordered_orm_classes = [node.name for node in LevelOrderIter(orm_tree)]
    for table in ordered_orm_classes:#Base.__subclasses__():  # iterate over all tables
        highest_free_key_value = 0
        primary_keys[table] = {}
        if table is pycram.orm.base.Base: # The baseclase has no table representation
            continue
        list_of_primary_keys_of_this_table = table.__table__.primary_key.columns.values()
        for key in list_of_primary_keys_of_this_table:
            # make it smart but maybe
            all_source_key_values = []
            all_destination_key_values = []
            for key_value_row in source_session.query(key).all():
                all_source_key_values.append(key_value_row[0])  # get all values of key from source session
            for key_value_row in destination_session.query(key).all():
                all_destination_key_values.append(key_value_row[0])  # get all values of key from source session
            primary_keys[table][key.name] = destination_session.query(key).all()
            if all_source_key_values:
                if all_destination_key_values:  # need to check if destination maybe has more items then source
                    if max(all_source_key_values) < max(all_destination_key_values):
                        highest_free_key_value = max(all_destination_key_values) + 1
                    else:
                        highest_free_key_value = max(all_source_key_values) + 1
                else:  # if destination values do not exist we use source
                    highest_free_key_value = max(all_source_key_values) + 1
            for column_object in destination_session.query(table).all():  # iterate over all columns
                if column_object.__dict__[key.name] in all_source_key_values:
                    print("Found primary_key collision in table {} value: {} max value in memory {}".format(table,
                                                                                                            column_object.__dict__[
                                                                                                                key.name],
                                                                                                            highest_free_key_value))
                    rospy.loginfo(
                        "Found primary_key collision in table {} value: {} max value in memory {}".format(table,
                                                                                                          column_object.__dict__[
                                                                                                              key.name],
                                                                                                          highest_free_key_value))
                    sqlalchemy.orm.attributes.set_attribute(column_object, key.name, highest_free_key_value)
                    highest_free_key_value += 1
        destination_session.commit()  # commit after every table
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
    objects_to_add = []
    destination_session = destination_session_maker()
    try:
        orm_tree = get_tree(pycram.orm.base.Base)
        ordered_orm_classes = [node.name for node in LevelOrderIter(orm_tree)]
        for orm_object_class in ordered_orm_classes:#Base.__subclasses__():
            if orm_object_class is pycram.orm.base.Base:  # The baseclase has no table representation
                continue
            objects_to_add = []
            result = source_session.execute(
                sqlalchemy.select(orm_object_class).options(sqlalchemy.orm.joinedload('*'))).mappings().all()
            for row in result:
                for key in row:
                    if not sqlalchemy.inspect(row[key]).detached and not sqlalchemy.inspect(
                            row[key]).transient and not sqlalchemy.inspect(row[key]).deleted:
                        source_session.refresh(row[key])  # get newest value
                        source_session.expunge(row[key])
                        sqlalchemy.orm.make_transient(row[key])
                        objects_to_add.append(row[key])
                    else:
                        rospy.logwarn("WARNING: Ignored already detached ORM Object {} ".format(
                            row[key]))

            if len(objects_to_add) < 1:
                return
            destination_session.add_all(objects_to_add)
            destination_session.commit()

    except Exception as e:
        traceback.print_exc()
    finally:
        source_session.close()
        destination_session.close()


def update_primary_key_constrains(session_maker: sqlalchemy.orm.sessionmaker, orm_classes: list):
    '''
    Iterates through the list of all ORM Classes and sets in their corresponding tables all foreign keys in the given
    endpoint to on update cascading. Careful currently only works on postgres databases.
    :param session_maker:
    :param orm_classes:
    :return: empty
    '''
    with session_maker() as session:
        for orm_class in orm_classes:
            try:
                foreign_key_statement = sqlalchemy.text(
                    "SELECT con.oid, con.conname, con.contype, con.confupdtype, con.confdeltype, con.confmatchtype, pg_get_constraintdef(con.oid) FROM pg_catalog.pg_constraint con INNER JOIN pg_catalog.pg_class rel ON rel.oid = con.conrelid INNER JOIN pg_catalog.pg_namespace nsp ON nsp.oid = connamespace WHERE rel.relname = '{}';".format(
                        orm_class.__tablename__))
                response = session.execute(foreign_key_statement)  # engine.connect().execute(foreign_key_statement)
                print(25 * '~' + "{}".format(orm_class.__tablename__) + 25 * '~')
                for line in response:
                    if line.conname.endswith("fkey"):
                        if 'a' in line.confupdtype:  # a --> no action | if there is no action we set it to cascading
                            # I just assume there aren't any other constraints
                            drop_statement = sqlalchemy.text(
                                "alter table \"{}\" drop constraint \"{}\";".format(orm_class.__tablename__,
                                                                                    line.conname))
                            drop_response = session.execute(
                                drop_statement)  # There is no real data coming back for this
                            alter_statement = sqlalchemy.text(
                                "alter table \"{}\" add constraint {} {} on update cascade;".format(
                                    orm_class.__tablename__,
                                    line.conname,
                                    line.pg_get_constraintdef))
                            alter_response = session.execute(
                                alter_statement)  # There is no real data coming back for this
                            session.commit()
            except AttributeError:
                print("Attribute Error: {} has no attribute __tablename__".format(orm_class))
