cmake_minimum_required(VERSION 2.8.3)
project(pycram_tasktree_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  pycram
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#catkin_python_setup()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
