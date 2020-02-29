cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sgtdv_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/PathPlanningNODE.cpp src/PathPlanning.cpp src/PathPlanningSynch.cpp src/PathPlanningAlgorithms.cpp)

target_link_libraries(path_planning
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
