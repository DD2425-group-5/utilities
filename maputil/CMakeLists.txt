cmake_minimum_required(VERSION 2.8.3)
project(maputil)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  mapping_msgs
  geometry_msgs
  mathutil
)

catkin_package(
  LIBRARIES maputil
  DEPENDS roscpp nav_msgs mapping_msgs geometry_msgs mathutil
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(include ${catkin_INCLUDE_DIRS})

ADD_LIBRARY(maputil src/maputil.cpp)
