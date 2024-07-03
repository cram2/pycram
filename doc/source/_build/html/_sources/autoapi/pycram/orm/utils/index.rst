:py:mod:`pycram.orm.utils`
==========================

.. py:module:: pycram.orm.utils


Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   pycram.orm.utils.write_database_to_file
   pycram.orm.utils.print_database
   pycram.orm.utils.update_primary_key
   pycram.orm.utils.copy_database
   pycram.orm.utils.update_primary_key_constrains
   pycram.orm.utils.migrate_neems



.. py:function:: write_database_to_file(in_sessionmaker: sqlalchemy.orm.sessionmaker, filename: str, b_write_to_console: bool = False)

   Writes all Tables stored within the given session into a local file. File will be written in JSON Format

   :param in_sessionmaker: sessionmaker that allows us to access the Database
   :param filename: Filename of the logfile
   :param b_write_to_console: enables writing to the console. Default false


.. py:function:: print_database(in_sessionmaker: sqlalchemy.orm.sessionmaker)

   Prints all ORM Class data within the given Session.

   :param in_sessionmaker: Database Session which should be printed


.. py:function:: update_primary_key(source_session_maker: sqlalchemy.orm.sessionmaker, destination_session_maker: sqlalchemy.orm.sessionmaker)

   Updates all the primary keys of the database associated with the destination engine, so that there will be no
   problems when merging it into the source database. In order to achieve this the highest id value of the source
   engine is searched and the primary keys of the destination database will get all the values following that.
   Cascading triggers in the database will take care of the rest. Careful 2023 this will not work in
   memory databases as there are no triggers.

   :param source_session_maker: Session maker of the source data_base
   :param destination_session_maker: Session maker of the destination data_base


.. py:function:: copy_database(source_session_maker: sqlalchemy.orm.sessionmaker, destination_session_maker: sqlalchemy.orm.sessionmaker)

   Iterates through all tables within tht source database and merges them into the destination database. Careful
   this function does not check if there are any primary key collisions or updates any data.

    .. note::
       Ignores all previously detached data, could result in loss of information. During testing database objects
       sometimes had a detached twin. As a possible feature in the future it maybe useful to give the user an
       opportunity to decide what happens with the detached objects. Careful this could lead to duplicated data in the
       destination database.

   :param source_session_maker: Sessionmaker of the source database
   :param destination_session_maker: Sessionmaker of the destination database


.. py:function:: update_primary_key_constrains(session_maker: sqlalchemy.orm.sessionmaker)

   Iterates through all tables related to any ORM Class and sets in their corresponding foreign keys in the given
   endpoint to "ON UPDATE CASCADING".

       .. note::
           Careful currently only works on postgres databases.

   :param session_maker:
   :return: empty


.. py:function:: migrate_neems(source_session_maker: sqlalchemy.orm.sessionmaker, destination_session_maker: sqlalchemy.orm.sessionmaker)

   Merges the database connected to the source session maker into the database connected to the destination session
   maker. Will first update the primary constrains inside the destination database (if needed). Afterwards
   updates the primary keys within the destination database (as there are cascading updates now) and then merges
   the source database into the destination.

    .. note::
       Assumes the destination database is a postgres database

   :param source_session_maker: Sessionmaker of the source database
   :param destination_session_maker: Sessionmaker of the destination database


