# Исполняемые файлы
add_executable(obstacle_detector
        src/obstacle_detector/obstacle_detector.cpp)

# Подключение библиотек
target_link_libraries (obstacle_detector ${catkin_LIBRARIES})
target_compile_options(obstacle_detector PRIVATE -std=c++11)