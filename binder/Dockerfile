FROM intel4coro/base-notebook:20.04-noetic-full-xpra

# Set environment
ENV PATH=$PATH:/home/user/.local/bin
ENV PYCRAM_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${PYCRAM_WS}/src/
COPY --chown=${NB_USER}:users . pycram/
RUN vcs import --input pycram/binder/pycram-http.rosinstall --recursive

RUN pip install --requirement ${PYCRAM_WS}/src/pycram/binder/requirements.txt --user 

COPY --chown=${NB_USER}:users binder/me ${PYCRAM_WS}/src/me

USER root
RUN apt-get update

# Giskard
COPY --chown=${NB_USER}:users binder/giskard.rosinstall /home/${NB_USER}/giskard.rosinstall
RUN vcs import --input /home/${NB_USER}/giskard.rosinstall --recursive

# Giskard: requirements + missing packages
RUN pip install --requirement ${PYCRAM_WS}/src/giskardpy/requirements.txt
RUN pip install termcolor pydot

# Giskard: Pybind + betterpybullet
WORKDIR /home/${NB_USER} 
RUN git clone https://github.com/pybind/pybind11.git \
  && cd pybind11 \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=3 -DPYBIND11_TEST=OFF
USER root
RUN cd pybind11/build && make install
USER ${NB_USER}

RUN git clone https://github.com/SemRoCo/bullet3.git
COPY --chown=${NB_USER}:users binder/build_cmake_pybullet_3.10_double.sh /home/${NB_USER}/bullet3/
RUN cd bullet3 && ./build_cmake_pybullet_3.10_double.sh
ENV PYTHONPATH=${PYTHONPATH}:/home/${NB_USER}/bullet3/build_cmake/better_python:/home/${NB_USER}/bullet3/examples/pybullet


# HSR resources
WORKDIR ${PYCRAM_WS}/src/
RUN git clone https://github.com/code-iai/iai_hsr.git
WORKDIR  ${PYCRAM_WS}/src/iai_hsr/
RUN git submodule update --init

# Stretch resources
WORKDIR ${PYCRAM_WS}/src/
RUN git clone https://github.com/hello-robot/stretch_ros.git
RUN git clone https://github.com/issaiass/realsense2_description.git

# Tiago resources
WORKDIR ${PYCRAM_WS}/src/
RUN git clone https://github.com/code-iai/iai_tiago.git
WORKDIR  ${PYCRAM_WS}/src/iai_tiago/
RUN git submodule update --init

# Build pycram workspace
WORKDIR  ${PYCRAM_WS}
USER root
RUN rosdep update \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
USER ${NB_USER}
RUN catkin build

RUN pip install service_identity
RUN pip install netifaces

# Jupyter Config
COPY --chown=${NB_USER}:users binder/jupyter-config.json /opt/conda/share/jupyter/lab/settings/overrides.json

WORKDIR ${PYCRAM_WS}/src/pycram
RUN git config --global --add safe.directory ${PWD}
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter", "lab", "--allow-root", "--NotebookApp.token=''", "--no-browser", "--ip=0.0.0.0"]
RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.1.tar.gz
