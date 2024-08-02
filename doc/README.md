# Documentation
This directory contains the whole documentation for PyCRAM. To build the documentation please follow
the instructions below.



## Building the documentation


The documentation uses jupyter-book as engine. Building the documentation requires Python 3.9 or higher to avoid 
dependency conflicts. On Ubuntu 20.04 you can install Python 3.9 with the following commands.
~~~
apt-get install python3.9
~~~

It is recommended to create a virtual environment to avoid conflicts with the system python interpreter.
~~~
apt-get install python3.9-virtualenv
virtualenv -p python3.9 --system-site-packages build-doc
~~~

Activate the virtual environment.
~~~
source build-doc/bin/activate
~~~


Install the requirements in your python interpreter.

~~~
pip install -r requirements.txt
~~~
Run pycram and build the docs.

~~~
cd doc/source 
jupyter-book build .
~~~
Show the index.

~~~
firefox _build/html/index.html
~~~