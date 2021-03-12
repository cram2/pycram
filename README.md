# PyCRAM

Python 3 implementation of CRAM.

## Authors

* **Andy Augsten** <a.augsten@uni-bremen.de>
* **Dustin Augsten** <augsten@uni-bremen.de>
* **Jonas Dech** <jdech@uni-bremen.de>
* **Christopher Pollok** <cpollok@uni-bremen.de>
* **Thomas Lipps** <tlipps@uni-bremen.de>

## Dependecies
To work with the Bullet World of PyCRAM the follwing packages are required.
* PyBullet
* pathlib
* numpy
* rospkg 
* atexit
* urdfpy
* graphviz 

### Macropy

Additionally a specific fork of macropy is needed, which is already included in this repo as a submodule.
To get this submodule just run the following commands in the Repo.
```
git submodule init
```

```
git submodule update
```

## Run the Demo
### Prerequisites
To run the demo one needs, all files in the resources folder along with the [pr2 description](https://github.com/PR2/pr2_common/tree/melodic-devel/pr2_description) and the description of the [kitchen](https://github.com/code-iai/iai_maps/tree/master/iai_kitchen). Both descriptions should be placed in the resource directory.

### Execute
To run the demo the following command has to be executed:

```
python3 <path-to-demo-directory>/run.py
```
