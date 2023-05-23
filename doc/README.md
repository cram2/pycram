# Documentation
This directory contains the whole documentation for PyCRAM. To build the documentation please follow
the instructions below.



## Building the documentation


The documentation uses sphinx as engine.
Building sphinx based documentations requires `pandoc <https://pandoc.org/installing.html>`_
to be installed. Pandoc can be installed via the Ubunutu package manager:
~~~
sudo apt install pandoc
~~~
After installing pandoc, install sphinx on your device.

~~~
sudo apt install python3-sphinx
~~~
Install the requirements in your python interpreter.

~~~
pip install -r requirements.txt
~~~
Run pycram and build the docs.

~~~
make html
~~~
Show the index.

~~~
firefox build/html/index.html
~~~