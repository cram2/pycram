=========================
Jupyter Notebook Examples
=========================

PyCRAM uses Jupyter Notebooks as a means of providing code examples and tutorials. These examples are located in the
examples directory.

These Notebooks are stored as Myst Markdown files, which provide the advantage of better verison control and easier
integration with Sphinx documentation. However, the Notebooks can be run as usual in Jupyter Notebook.


================
Running Examples
================
Despite the examples being stored as Myst Markdown files, they can still be run as Jupyter Notebooks. To run an example,
simply open the Notebook in Jupyter Notebook and run the cells as you would with any other Notebook.

To run Jupyter Notebook, you need to have Jupyter Notebook installed. You can install Jupyter Notebook using pip:

.. code-block:: bash

    pip install jupyter

After installing Jupyter Notebook, you can run it by executing the following command in the terminal:

.. code-block:: bash

    jupyter notebook

Afterwards, a new website will open in your browser, where you can navigate to the examples directory and open the
desired Notebook.


===============
Adding Examples
===============
If you want to add a new example, you can do so by creating a new Notebook in the examples directory. After creating
and testing the example you need to convert it to the Myst Markdown format. For this step we use the
`jupytext <https://github.com/mwouts/jupytext>`_ tool. To convert a Notebook to Myst Markdown, run the following
command in the examples directory of the project:

.. code-block:: bash

    jupytext --to markdown your_notebook.ipynb

-------------------------------
Linking directly to source code
-------------------------------
Since the examples are stored as Myst Markdown files, you can link directly to the source code documentation. For
example, to link to the source code of the class `PickUpAction` you can use:

.. code-block:: markdown

    {class}`~pycram.designators.action_designator.PickUpAction`


Alternatively, you can use `{meth}`, `{func}`, `{attr}`, `{class}`, `{mod}` and `{obj}` to link to methods, functions,
attributes, classes, modules and objects, respectively.
