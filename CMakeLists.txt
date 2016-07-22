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
  message_generation
)

find_package(PCL 1.2 REQUIRED)

# Сервисы
add_service_files(
        FILES
        StepsController.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

## Declare a catkin package
catkin_package()


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "")

include_directories(${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        src)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#TODO: Добавлять исходники в CMakeList.txt как-то более удобно
add_executable(steps_controller
        src/steps_controller/steps_controller.cpp
        src/steps_controller/CloudTransforms/CloudTransforms.cpp
        src/steps_controller/StepsController/StepsController.cpp
        src/steps_controller/StepsController/StepsParams.h
        src/steps_controller/ExtendedMath/ExtendedMath.cpp)
target_link_libraries (steps_controller ${PCL_LIBRARIES} ${catkin_LIBRARIES})