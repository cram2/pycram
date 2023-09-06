# Pycram on BinderHub

Files for running Pycram on BinderHub.

## Quick Start

### Option 1: Test Image Locally (Under repo directory)

- Run Docker image with X-forwarding

  ```bash
  ./binder/run_local.sh
  ```

- Open Web browser and go to http://localhost:8888/

- Force rebuilding image

  ```bash
  docker build ./ -f ./binder/Dockerfile -t pycram:binder
  ```

### Option 2: Run on BinderHub

- Link to the binderhub: https://binder.intel4coro.de/v2/gh/IntEL4CoRo/pycram/binder

## Usage

1. Go to example directory

1. Docs: https://pycram.readthedocs.io/en/latest/examples.html

## Files Descriptions

1. ***[Dockerfile](./Dockerfile):*** Pycram jupyterlab docker image.
1. ***[pycram-http.rosinstall](./pycram-http.rosinstall):*** Initiating ros workspace in docker image require https url. (Comparing to [pycram.rosinstall](../pycram.rosinstall): pycram is excluded, a new repo `orocos_kinematics_dynamics` is PyKDL).
1. ***[entrypoint.sh](./entrypoint.sh):*** Entrypoint of the docker image, start roscore and visualization tools.
1. ***[docker-compose.yml](./docker-compose.yml):*** For testing the docker image locally.
1. ***[run_local.sh](./run_local.sh):*** For testing the docker image locally.
