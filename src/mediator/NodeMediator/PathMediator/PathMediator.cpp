//
// Created by garrus on 21.02.17.
//

#include "PathMediator.h"

PathMediator::PathMediator(ros::NodeHandle nh, int maxBufferSize)
        : NodeMediatorBase(maxBufferSize)
{
    ros::Subscriber path_subscriber = nh.subscribe("/footstep_planner/path", 1, &PathMediator::Callback, this);
    ros::Publisher goal_publisher = nh.advertise<geometry_msgs::PoseStamped>("goal", 1, true);
}

//Преобразует массив double в нужные параметры
void PathMediator::_SendRequest(const double* buffer, int count)
{
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = buffer[0];
    goal.pose.position.y = buffer[0];
    // fixme work with orientation. while now it's zero

    goal_publisher.publish(goal);
}

//Расчет тракетории законечен
void PathMediator::Callback(nav_msgs::Path path)
{
    //TODO: по-идее, в С++ 14 можно захватывать переменные класса
    int bufferMaxSize = BufferMaxSize;
    Done([&path, bufferMaxSize](double* buffer, int & dataSize)
    {
        //Преобразует ответ в массив double'во
        int steps_count = path.poses.size();

        //Под буфер требуется 2*steps_count+1
        if(bufferMaxSize < 2 * steps_count + 1)
            throw std::overflow_error("Inner buffer is too small, to contains all path points");

        buffer[0] = steps_count;
        for (int i = 0; i < steps_count; i++)
        {
            buffer[1 + 2*i] = path.poses[i].pose.position.x;
            buffer[1 + 2*i + 1] = path.poses[i].pose.position.y;
            // TODO are we need their orientation or something like that?
        }
    });
}