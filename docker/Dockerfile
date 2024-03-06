ARG FROM_IMAGE=ros:noetic-ros-core
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM $FROM_IMAGE AS cacher

RUN sudo apt-get update && apt-get install python3-pip python3-vcstool git -y
RUN vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram-https.rosinstall --recursive --skip-existing $OVERLAY_WS/src

RUN pip3 install rosdep && rosdep init
RUN rosdep update && rosdep install --from-paths $OVERLAY_WS/src --ignore-src -r -y

RUN . /opt/ros/noetic/setup.sh && cd $OVERLAY_WS && catkin_make
RUN echo "source $OVERLAY_WS/devel/setup.bash" >> ~/.bashrc

EXPOSE 11311

# WORKDIR $OVERLAY_WS/src/pycram
# RUN sudo pip3 install --upgrade pip --root-user-action=ignore
# RUN sudo pip3 install -r requirements.txt
# RUN pip3 install --ignore-installed pytest pyjpt mlflow
