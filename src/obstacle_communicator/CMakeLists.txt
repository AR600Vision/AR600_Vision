FIND_PACKAGE(OpenMP)

#add_definitions(-std=c++11) # it's for std::mutex support

add_executable(obstacle_communicator
        src/obstacle_communicator/obstacle_communicator.cpp)

target_link_libraries(obstacle_communicator  ${catkin_LIBRARIES})
target_compile_options(obstacle_communicator PRIVATE -std=c++11)
