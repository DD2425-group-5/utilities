cmake_minimum_required(VERSION 2.8.3)
project(sysutil)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  LIBRARIES sysutil
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ADD_LIBRARY(sysutil src/sysutil.cpp)
