from sqlalchemy.orm import Session
import tqdm
from typing_extensions import Optional
from .model import TaskTreeNode as TaskTreeNodeORM, self_mapped_classes, explicitly_mapped_classes
from ..tasktree import TaskTreeNode


def insert(node: TaskTreeNode, session: Session, use_progress_bar: bool = True,
           progress_bar: Optional[tqdm.tqdm] = None, parent: Optional[TaskTreeNodeORM] = None):

    if use_progress_bar:
        if not progress_bar:
            progress_bar = tqdm.tqdm(desc="Inserting TaskTree into database", leave=True, position=0, total=len(node))

    # convert self to orm object
    orm_node = TaskTreeNodeORM(start_time=node.start_time, end_time=node.end_time, status=node.status, reason=str(node.reason))


    # check for explicit mappings
    is_explicit_mapping = False
    for explicit_class in explicitly_mapped_classes:
        if type(node.action) == explicit_class.explicit_mapping:
            is_explicit_mapping = True
            break

    # add the action to the node
    if type(node.action) in self_mapped_classes or is_explicit_mapping:
        orm_node.action = node.action
    else:
        orm_node.action = None

    # set node parent
    orm_node.parent = parent

    # add the node to the session; note that the instance is not yet committed to the db, but rather in a
    session.add(orm_node)

    if progress_bar:
        progress_bar.update()

    # if recursive, insert all children
    [insert(child, session, use_progress_bar=use_progress_bar, progress_bar=progress_bar, parent=orm_node,)
     for child in node.children]

    # once recursion is done and the root node is reached again, commit the session to the database
    if node.parent is None:
        session.commit()
