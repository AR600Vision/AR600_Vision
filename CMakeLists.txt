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
        )

# Дополнительные зависимости
find_package(pcl_conversions)
find_package(PCL 1.2 REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        src)


# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Ноды
include(${CMAKE_CURRENT_SOURCE_DIR}/src/steps_controller/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/mediator/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/path_controller/CMakeLists.txt)

# Сервисы
#add_service_files(
#        FILES
#        StepsController.srv
#)

add_message_files(
        FILES
        StepResponse.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

# Declare a catkin package
catkin_package()


set(CMAKE_CXX_FLAGS "-Wno-deprecated")