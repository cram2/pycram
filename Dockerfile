
ARG FROM_IMAGE=ros:noetic-ros-core-focal
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM $FROM_IMAGE as builder
ARG OVERLAY_WS=/opt/ros/overlay_ws
WORKDIR $OVERLAY_WS/src

RUN sudo apt-get update && apt-get install python3-pip python3-vcstool git default-jre -y
RUN vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram-https.rosinstall --recursive --skip-existing $OVERLAY_WS/src
RUN sudo apt-get update && apt-get install python3-pip -y && pip3 install pip --upgrade
RUN pip3 install --no-cache-dir -r $OVERLAY_WS/src/pycram/requirements.txt

RUN pip3 install rosdep && rosdep init
RUN rosdep update && rosdep install --from-paths $OVERLAY_WS/src --ignore-src -r -y
RUN sudo apt-get install jupyter -y

RUN . /opt/ros/noetic/setup.sh && cd $OVERLAY_WS && catkin_make
RUN echo "source $OVERLAY_WS/devel/setup.bash" >> ~/.bashrc
RUN rm -rf $OVERLAY_WS/src/pycram

COPY . $OVERLAY_WS/src/pycram
RUN pip3 install -r $OVERLAY_WS/src/pycram/requirements.txt
COPY docker/entrypoint.sh /
ENTRYPOINT ["bash", "/entrypoint.sh"]

# launch ros package
#CMD ["roslaunch", "pycram", "ik_and_description.launch", "&"]
#CMD ["jupyter", "notebook", "--ip", "0.0.0.0", "--allow-root", "/opt/ros/overlay_ws/src/pycram/examples/"]
