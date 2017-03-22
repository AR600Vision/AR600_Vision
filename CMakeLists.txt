cmake_minimum_required(VERSION 2.8.3)
project(ar600_vision)


# Подключение общих для всего пакета зависимостей
# Не надо подключать их повторно
# Некоторые зависимости не удается подключить только в ноде  - щито поделать
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs             # не подключается
        message_generation      # не подключается
        tf
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
        src)

# Ноды
include(${CMAKE_CURRENT_SOURCE_DIR}/src/steps_controller/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/mediator/CMakeLists.txt)


# Declare a catkin package
catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -fopenmp")
