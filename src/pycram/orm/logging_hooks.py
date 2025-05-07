from sqlalchemy.orm import Session
import tqdm
from typing_extensions import Optional
from .model import TaskTreeNode as TaskTreeNodeORM, self_mapped_classes, explicitly_mapped_classes
from ..plan import Plan


def insert(plan: Plan, session: Session, use_progress_bar: bool = True,
           progress_bar: Optional[tqdm.tqdm] = None):
    """
    Insert a plan into the database. Inserts all nodes and edges from the given plan into the database if the nodes and
    associated designators are present in the ORM model. Otherwise, check the genereate_orm.py file in scrip/ and the
    model.py file for explicitly mapped classes.

    :param plan: Plan to be inserted into the database.
    :param session: Session object for database interaction.
    :param use_progress_bar: If True, a progress bar will be displayed during the insertion process.
    :param progress_bar: Optional progress bar object. If not provided, a new one will be created.
    """

    if use_progress_bar:
        if not progress_bar:
            progress_bar = tqdm.tqdm(desc="Inserting Plan into database", leave=True, position=0, total=len(plan.nodes))

    # convert self to orm object
    for node in plan.nodes:
        session.add(node)

    session.commit()
