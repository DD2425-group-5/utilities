cmake_minimum_required(VERSION 2.8.3)
project(calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_arduino_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wallfollower
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES} rosutil)
#add_dependencies(calibration calibration_cpp)
add_dependencies(calibration ${catkin_EXPORTED_TARGETS})
