cmake_minimum_required(VERSION 2.8.3)
project(pclutil)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)
find_package(PCL REQUIRED)

catkin_package(
  LIBRARIES pclutil
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

ADD_LIBRARY(pclutil src/pclutil.cpp)
