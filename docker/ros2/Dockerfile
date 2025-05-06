# has to be called from the root dir of the repo with "sudo docker build -f docker/ros2/Dockerfile ." otherwise COPY will
# not work

ARG FROM_IMAGE=ros:jazzy
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM $FROM_IMAGE as builder
ARG OVERLAY_WS=/opt/ros/overlay_ws
WORKDIR $OVERLAY_WS/src
SHELL ["/bin/bash", "-c"]

RUN apt update
RUN apt install python3.12-venv ros-jazzy-xacro python3-vcstool git ros-dev-tools python3-colcon-common-extensions ros-jazzy-ament-cmake ros-jazzy-ament-package ros-jazzy-ament-cmake-core ros-jazzy-ament-cmake-clang-format -y
RUN vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram-ros2-https.rosinstall
RUN source /opt/ros/jazzy/setup.bash && cd $OVERLAY_WS && colcon build --symlink-install
RUN echo "source $OVERLAY_WS/install/setup.bash" >> ~/.bashrc

RUN rm -rf $OVERLAY_WS/src/pycram
COPY . $OVERLAY_WS/src/pycram
RUN python3 -m venv pycram/pycram-venv --system-site-packages  && source pycram/pycram-venv/bin/activate && pip install -U pip && pip install -U setuptools && pip install -r $OVERLAY_WS/src/pycram/requirements.txt

COPY docker/ros2/entrypoint.sh /
ENTRYPOINT ["bash", "/entrypoint.sh"]

