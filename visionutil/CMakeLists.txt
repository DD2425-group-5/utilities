cmake_minimum_required(VERSION 2.8.3)
project(visionutil)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES visionutil
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ADD_LIBRARY(visionutil src/visionmodels.cpp src/visionmath.cpp src/clustering.cpp)# src/geometry.cpp)
