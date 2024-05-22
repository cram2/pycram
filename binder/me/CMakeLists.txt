cmake_minimum_required(VERSION 3.0.2)
project(me)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND rm -rf jovyan
)

execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND mkdir -p jovyan/workspace/ros/
)

execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND ln -s /home/jovyan/workspace/ros/src jovyan/workspace/ros/src
)

