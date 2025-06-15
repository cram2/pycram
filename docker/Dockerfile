ARG FROM_IMAGE=ros:noetic-ros-core
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM $FROM_IMAGE AS cacher

ARG OVERLAY_WS

RUN apt-get update && apt-get install python3-pip python3-vcstool git default-jre python3-catkin-tools ros-$ROS_DISTRO-xacro -y && pip3 install pip --upgrade
RUN pip3 install rosdep && rosdep init

WORKDIR $OVERLAY_WS/src
COPY . pycram
RUN vcs import --skip-existing $OVERLAY_WS/src < pycram/rosinstall/pycram-https.rosinstall
RUN rosdep update && rosdep install --from-paths $OVERLAY_WS/src --ignore-src -r -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && cd $OVERLAY_WS && catkin build
RUN echo "source $OVERLAY_WS/devel/setup.bash" >> ~/.bashrc

RUN pip3 install --upgrade pip
WORKDIR $OVERLAY_WS/src/pycram
RUN pip3 install -r requirements.txt

EXPOSE 11311
