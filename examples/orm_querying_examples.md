---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.3
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

# ORM querying 

Querying the ORM can be done by writing custom queries using sqlalchemy. 
However, if you regulary want to access a chunk of data, it is recommended to write a view that does this for you.
You can see the details of writing a view in here :py:func:`pycram.orm.base.views.PickUpWithContextView`.