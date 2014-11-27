cmake_minimum_required(VERSION 2.8.3)
project(mathutil)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  LIBRARIES mathutil
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ADD_LIBRARY(mathutil src/mathutil.cpp)
