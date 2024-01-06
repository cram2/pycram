FROM ros:noetic

ARG OVERLAY_WS=/opt/ros/overlay_ws
WORKDIR $OVERLAY_WS/src

COPY . $OVERLAY_WS/src/pycram
RUN sudo apt-get update && apt-get install python3-pip python3-vcstool git -y
RUN ls
RUN vcs import --input $OVERLAY_WS/src/pycram/pycram-https.rosinstall --recursive --skip-existing $OVERLAY_WS/src
RUN rosdep update && rosdep install --from-paths $OVERLAY_WS/src --ignore-src -r -y

RUN sudo apt-get update && apt-get install python3-pip -y && pip3 install --no-cache-dir -r $OVERLAY_WS/src/pycram/requirements.txt
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd $OVERLAY_WS; catkin_make'

# launch ros package
CMD ["/bin/bash", "-c", ". $OVERLAY_WS/devel/setup.bash"]
CMD ["roslaunch", "pycram", "ik_and_description.launch"]
