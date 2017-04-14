#find_package(geometry_msgs)

#add_definitions(-std=c++11) # it's for std::mutex support

add_executable(mediator_node
        src/mediator/mediator_node.cpp
        src/mediator/NodeMediator/NodeMediatorBase.cpp
        src/mediator/NodeMediator/StepsMediator/StepsMediator.cpp
        src/mediator/NodeMediator/PathMediator/PathMediator.cpp
        src/mediator/NodeMediator/ObstacleMediator/ObstacleMediator.cpp)

target_link_libraries(mediator_node  ${catkin_LIBRARIES})

target_compile_options(mediator_node PRIVATE -std=c++11)

add_executable(sockettest src/mediator/sockettest.cpp)

target_compile_options(sockettest PRIVATE -std=c++11)