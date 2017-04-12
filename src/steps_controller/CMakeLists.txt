# Нода steps_controller

# Дополнительные зависимости
find_package(pcl_conversions)
find_package(PCL 1.2 REQUIRED)

# Сервисы
#add_service_files(
#        FILES
#        StepsController.srv
#)



# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Исполняемые файлы
add_executable(steps_controller     #имя результирующего файла - ноды
        src/steps_controller/steps_controller.cpp
        src/steps_controller/CloudTransforms/CloudTransforms.cpp
        src/steps_controller/StepsController/StepsController.cpp
        src/steps_controller/StepsController/StepsParams.h
        src/steps_controller/Math/Math.cpp src/steps_controller/Utils/Utils.cpp src/steps_controller/Utils/Utils.h src/steps_controller/Utils/DataAccessFunctor.cpp src/steps_controller/Utils/DataAccessFunctor.h src/steps_controller/StepsController/FootTargetFunctor.cpp src/steps_controller/StepsController/FootTargetFunctor.h)

# Подключение библиотек
# Внимание - надо подключать ${catkin_LIBRARIES} для каждого испольняемого файла
target_link_libraries (steps_controller ${PCL_LIBRARIES} ${catkin_LIBRARIES})