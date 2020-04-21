# PyCRAM

Python 3 implementation of CRAM.

PyCRAM requires [a specific fork of MacroPy](https://github.com/daugsten93/macropy), which, however, is already included in the source. So, you do not have to install it.

## Authors

* **Andy Augsten** <a.augsten@uni-bremen.de>
* **Dustin Augsten** <augsten@uni-bremen.de>
* **Jonas Dech** <jdech@uni-bremen-de>

## Dependecies 
To work with the Bullet World of PyCRAM the follwing packages are required. 
* PyBullet
* pathlib 
* numpy

## Run the Demo 
### Prerequisites
To run the demo one needs, all files in the resources folder along with the [pr2 description](https://github.com/PR2/pr2_common/tree/melodic-devel/pr2_description) and the description of the [kitchen](https://github.com/code-iai/iai_maps/tree/master/iai_kitchen). **Both descriptions must be in the same folder as this repository**.

### Execute
To execute the demo the follwing commands have to be executed:
```
cd demo
```
```
python3 run.py
```
