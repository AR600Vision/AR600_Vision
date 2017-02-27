# path controller
# Dependencies:
#   RTABMAP
#   Humanoid navigation
#       SBPL

find_package(catkin REQUIRED COMPONENTS
        footstep_planner
        rtabmap_ros)

# Sbpl package
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})


add_definitions(-std=c++11)
add_executable(path_controller src/path_controller/path_controller.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(path_controller
        ${catkin_LIBRARIES} ${SBPL_LIBRARIES}
        )