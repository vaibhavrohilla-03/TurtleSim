cmake_minimum_required(VERSION 3.0.2)
project(turtle_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  turtlesim
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_control src/main.cpp)
target_link_libraries(turtle_control ${catkin_LIBRARIES})