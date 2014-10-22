cmake_minimum_required(VERSION 2.8.3)
project(rosutil)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  LIBRARIES rosutil
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ADD_LIBRARY(rosutil src/rosutil.cpp)
