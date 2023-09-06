FROM intel4coro/intel4coro:jupyterhub-ros-base-0.5

ENV PATH=$PATH:/home/user/.local/bin
ENV PYCRAM_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${PYCRAM_WS}/src/
COPY --chown=${NB_USER}:users . pycram/
RUN pip install vcstool \
  && vcs import --input pycram/binder/pycram-http.rosinstall --recursive

# === Following step should be replace to ===
RUN cd pycram \
  && git submodule update --init \
  && git clone https://github.com/Tigul/neem_interface_python.git src/neem_interface_python \
  && cd src/neem_interface_python \
  && git clone https://github.com/benjaminalt/neem-interface.git src/neem-interface
# === To ===
# RUN cd pycram
  # && git submodule update --init --recursive
# === When all .gitmodules use https urls ===

RUN pip install --requirement ${PYCRAM_WS}/src/pycram/requirements.txt --user \
  && pip install --requirement ${PYCRAM_WS}/src/pycram/src/neem_interface_python/requirements.txt --user \
  && pip cache purge

COPY --chown=${NB_USER}:users binder/me ${PYCRAM_WS}/src/me

# Build pycram workspace
WORKDIR  ${PYCRAM_WS}
USER root
RUN rosdep update \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
USER ${NB_USER}
RUN catkin build

WORKDIR ${PYCRAM_WS}
RUN git config --global --add safe.directory ${PWD}
COPY --chown=${NB_USER}:users binder /binder
ENTRYPOINT ["/binder/entrypoint.sh"]
CMD ["start-notebook.sh"]