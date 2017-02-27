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
    if(count<2)
    {
        ROS_ERROR("PathMediator: not enough arguments in request");
        throw std::invalid_argument("not enough arguments in request");
    }

    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = buffer[0];
    goal.pose.position.y = buffer[0];
    // fixme work with orientation. while now it's zero

    ROS_INFO("path planner is callled");
    goal_publisher.publish(goal);
}

//Расчет тракетории законечен
void PathMediator::Callback(nav_msgs::Path path)
{
    Done([&path](double* buffer, int & dataSize, int maxSize)
    {
        //Преобразует ответ в массив double'во
        int steps_count = path.poses.size();

        //Под буфер требуется 2*steps_count+1
        if(maxSize < 2 * steps_count + 1)
        {
            ROS_ERROR("PathMediator: Inner buffer is too small to contains all result data");
            throw std::overflow_error("Inner buffer is too small to contains all result data");
        }

        buffer[0] = steps_count;
        for (int i = 0; i < steps_count; i++)
        {
            buffer[1 + 2*i] = path.poses[i].pose.position.x;
            buffer[1 + 2*i + 1] = path.poses[i].pose.position.y;
            // TODO are we need their orientation or something like that?
        }
    });
}