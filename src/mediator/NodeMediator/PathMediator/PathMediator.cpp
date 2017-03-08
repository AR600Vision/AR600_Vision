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
void PathMediator::SendRequest(double posX, double posY)
{
    std::lock_guard<std::mutex> l(Mutex);

    if(!isDone)
        return;

    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = posX;
    goal.pose.position.y = posY;
    // fixme work with orientation. while now it's zero

    goal_publisher.publish(goal);

    ROS_INFO("path_controller is called");

    isDone = false;
}

//Расчет тракетории законечен
void PathMediator::Callback(nav_msgs::Path path)
{

    //Преобразует ответ в массив double'во
    int steps_count = path.poses.size();

    //Под буфер требуется 2*steps_count+1
    if(BufferMaxSize < 2 * steps_count + 1)
    {
        ROS_ERROR("PathMediator: Inner buffer is too small to contains all result data");
        throw std::overflow_error("Inner buffer is too small to contains all result data");
    }

    Mutex.lock();

    SendBuffer[0] = steps_count;
    for (int i = 0; i < steps_count; i++)
    {
        SendBuffer[1 + 2*i] = path.poses[i].pose.position.x;
        SendBuffer[1 + 2*i + 1] = path.poses[i].pose.position.y;
        // TODO are we need their orientation or something like that?
    }


    Mutex.unlock();
}