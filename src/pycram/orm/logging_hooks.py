from sqlalchemy.orm import Session
import tqdm
from typing_extensions import Optional
from .model import TaskTreeNode as TaskTreeNodeORM, self_mapped_classes, explicitly_mapped_classes
from ..plan import Plan


def insert(plan: Plan, session: Session, use_progress_bar: bool = True,
           progress_bar: Optional[tqdm.tqdm] = None):

    if use_progress_bar:
        if not progress_bar:
            progress_bar = tqdm.tqdm(desc="Inserting Plan into database", leave=True, position=0, total=len(plan.nodes))

    # convert self to orm object
    for node in plan.nodes:
        session.add(node)

    session.commit()