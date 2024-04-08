FROM intel4coro/base-notebook:20.04-noetic-vnc

# Initiate workspace 
ENV PYCRAM_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${PYCRAM_WS}/src/

# Copy everything to the ROS workspace
COPY --chown=${NB_USER}:users . ${PYCRAM_WS}/src/pycram

# Can not use the pycram-https.rosinstall under the root directory, because some submodules are linked with SSH urls
RUN vcs import --input ${PYCRAM_WS}/src/pycram/binder/pycram-https.rosinstall --recursive

# Install python dependencies
WORKDIR ${PYCRAM_WS}/src/pycram
RUN pip install --requirement src/neem_interface_python/requirements.txt \
  && pip install --requirement requirements.txt

# Install ROS dependencies
WORKDIR  ${PYCRAM_WS}
USER root
RUN rosdep update && apt update && apt dist-upgrade -y \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions

# Build catkin workspace
USER ${NB_USER}
RUN catkin build

# JupyterLab UI configuration
WORKDIR ${PYCRAM_WS}/src/pycram
RUN git config --global --add safe.directory ${PWD}
COPY --chown=${NB_USER}:users binder/webapps.json ${PYCRAM_WS}/src/rvizweb/webapps/app.json

# Override container entrypoint
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
