cmake_minimum_required(VERSION 2.8.3)
project(ar600_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
)

find_package(PCL 1.2 REQUIRED)

## Declare a catkin package
catkin_package()


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "")

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#set(SOURCE_FILES main.cpp)
add_executable(steps_controller src/steps_controller.cpp)
target_link_libraries (steps_controller ${PCL_LIBRARIES} ${catkin_LIBRARIES})