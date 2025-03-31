ARG FROM_IMAGE=ros:noetic-ros-core
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM $FROM_IMAGE as cacher
ARG OVERLAY_WS 
WORKDIR $OVERLAY_WS/src

RUN apt-get update && apt-get install python3-pip python3-vcstool git default-jre python3-catkin-tools -y && pip3 install pip --upgrade
RUN pip3 install rosdep && rosdep init

RUN vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram-https.rosinstall --skip-existing $OVERLAY_WS/src
RUN rosdep update && rosdep install --from-paths $OVERLAY_WS/src --ignore-src -r -y

RUN . /opt/ros/noetic/setup.sh && cd $OVERLAY_WS && catkin build
RUN echo "source $OVERLAY_WS/devel/setup.bash" >> ~/.bashrc

RUN pip3 install --upgrade pip 
WORKDIR $OVERLAY_WS/src/pycram
RUN pip3 install -r requirements.txt

EXPOSE 11311
