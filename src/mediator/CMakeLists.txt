#find_package(geometry_msgs)

add_definitions(-std=c++11) # it's for std::mutex support
add_executable(mediator_node
        src/mediator/mediator_node.cpp
        src/mediator/NodeMediator/NodeMediatorBase.cpp
        src/mediator/NodeMediator/StepsMediator/StepsMediator.cpp
        src/mediator/NodeMediator/PathMediator/PathMediator.cpp)

target_link_libraries(mediator_node  ${catkin_LIBRARIES})

add_executable(sockettest
        src/mediator/sockettest.cpp)
