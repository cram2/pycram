#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd ../examples
rm -rf tmp
mkdir tmp
jupytext --to notebook *.md
mv *.ipynb tmp
cd tmp
treon --thread 1 -v --exclude=migrate_neems.ipynb --exclude=improving_actions.ipynb