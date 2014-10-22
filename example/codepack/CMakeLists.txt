cmake_minimum_required(VERSION 2.8.3)
project(codepack)

find_package(catkin REQUIRED COMPONENTS roscpp libfunk)

catkin_package(
  CATKIN_DEPENDS roscpp libfunk
)

# includes the catkin_ws/devel/include directory (I think?)
include_directories(include)

add_executable(code src/code.cpp)
target_link_libraries(code ${catkin_LIBRARIES} mylib tt)
add_dependencies(code ${catkin_EXPORTED_TARGETS})
