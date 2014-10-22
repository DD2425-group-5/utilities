cmake_minimum_required(VERSION 2.8.3)
project(libfunk)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  CATKIN_DEPENDS roscpp
  LIBRARIES mylib
  INCLUDE_DIRS src
)

add_library(mylib src/lib.cpp src/lib.hpp)
add_library(tt src/tt.cpp src/tt.hpp)

message("CMake for lib")

# find all header files in the source directory and subdirectories, and group them
file(GLOB_RECURSE headers src/*.hpp)

# remove previous versions of files (not sure how to replace only if modified)
file(REMOVE_RECURSE ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})
# go through each of the matched files
foreach(header ${headers})
  message(${header})
  # replace the path, up to the point of the src directory, so that
  # we end up with just the path below the src directory
  string(REGEX REPLACE ".*${CMAKE_CURRENT_SOURCE_DIR}\\/src" "" belowsrc ${header})
  # match directories in the path
  # i.e. /foo/bar/item.hpp -> /foo/bar/
  string(REGEX MATCH ".*\\/" DIR ${belowsrc})
  # copy the header file to the include directory in catkin_ws/devel,
  # under the package name, and preserving subdirectory structure
  file(COPY ${header} DESTINATION ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}${DIR})
endforeach(header)