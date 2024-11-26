cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sgtdv_msgs
  roslib
  tf
  tf_conversions
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS sgtdv_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../
)

add_executable(
  ${PROJECT_NAME} 
  src/path_planning_node.cpp 
  src/path_planning.cpp 
  src/path_planning_ros.cpp 
  src/rrt_star.cpp
)

target_link_libraries(path_planning
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
