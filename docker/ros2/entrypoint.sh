#!/bin/bash

set -e

source /opt/ros/overlay_ws/install/setup.bash
source /opt/ros/overlay_ws/src/pycram/pycram-venv/bin/activate

exec "$@"