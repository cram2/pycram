#!/bin/bash
# Allows x-forwarding and starts docker

xhost +local:docker
docker compose -f ./binder/docker-compose.yml up
xhost -local:docker
