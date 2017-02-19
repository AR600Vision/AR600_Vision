find_package(geometry_msgs)

add_definitions(-std=c++11) # it's for std::mutex support
add_executable(mediator_node src/mediator/mediator_node.cpp)

target_link_libraries(mediator_node  ${catkin_LIBRARIES})
