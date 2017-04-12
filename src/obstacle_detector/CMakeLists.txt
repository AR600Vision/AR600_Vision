INCLUDE(FindPkgConfig)

PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)


# Исполняемые файлы
add_executable(obstacle_detector
        src/obstacle_detector/obstacle_detector.cpp
        src/obstacle_detector/SimpleDraw/SimpleDraw.cpp)

# Подключение библиотек
#target_include_directories(obstacle_detector ${SDL2_INCLUDE_DIR})
target_link_libraries (obstacle_detector ${SDL2_LIBRARIES}  ${catkin_LIBRARIES})
target_compile_options(obstacle_detector PRIVATE -std=c++11)
#target_compile_options(obstacle_detector PRIVATE -std=c++11)
