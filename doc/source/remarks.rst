===============
General Remarks
===============

Jupyter Notebooks & PyCharm
===========================

If you have the situation that you activate your venv that has rospy installed and launched
PyCharm from the console you'll run into trouble using jupyter notebooks. PyCharm allows you
to correctly set up the jupyter server, however the notebook will always use your
``/usr/bin/python`` as interpreter and you can't select the correct one in the drop down menu.

To fix this issue one has to execute

.. code-block:: shell

    python -m ipykernel install --user --name <kernel_name> --display-name "<Name_to_display>"

, eg.

.. code-block:: shell

    python -m ipykernel install --user --name pycram --display-name "pycram"

in your terminal. --name is the name of your virtual environment and --display-name is the name
that will display in the drop down menu of jupyter. After that, select the correct Python interpreter kernel (``pycram``) and
everything should work now.
Refer `here <https://www.jetbrains.com/help/pycharm/configuring-jupyter-notebook.html#resolving-kernel-mismatch-error-of-configured-server>`_ for details.


Dirty Terminals
===============

If your terminal gets polluted by PyBullet complaining about incomplete URDF descriptions, you need to first fix your
URDF files by inserting the missing tags and second delete the `resources/cached` folder.
