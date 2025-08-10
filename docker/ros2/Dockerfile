# has to be called from the root dir of the repo with "sudo docker build -f docker/ros2/Dockerfile ." otherwise COPY will
# not work

ARG FROM_IMAGE=ros:jazzy
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM $FROM_IMAGE as builder
ARG OVERLAY_WS=/opt/ros/overlay_ws
WORKDIR $OVERLAY_WS/src
SHELL ["/bin/bash", "-c"]

RUN apt update && apt install python3.12-venv ros-jazzy-xacro python3-vcstool git ros-dev-tools default-jre -y
RUN vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/rosinstall/pycram-ros2-https.rosinstall
RUN git clone -b ros-jazzy https://github.com/code-iai/iai_robots.git
RUN source /opt/ros/jazzy/setup.bash && cd $OVERLAY_WS && colcon build --symlink-install
RUN echo "source $OVERLAY_WS/install/setup.bash" >> ~/.bashrc

# RUN rm -rf $OVERLAY_WS/src/pycram
# COPY . $OVERLAY_WS/src/pycram
RUN python3 -m venv pycram-venv --system-site-packages  && source pycram-venv/bin/activate && pip install -U pip && pip install -U setuptools && pip install -r $OVERLAY_WS/src/pycram/requirements.txt

COPY entrypoint.sh /
ENTRYPOINT ["bash", "/entrypoint.sh"]

