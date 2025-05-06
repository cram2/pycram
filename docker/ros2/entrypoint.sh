#!/bin/bash

set -e

source /opt/ros/overlay_ws/devel/setup.bash
source /opt/ros/overlay_ws/src/pycram/pycram-venv/bin/activate

exec "$@"