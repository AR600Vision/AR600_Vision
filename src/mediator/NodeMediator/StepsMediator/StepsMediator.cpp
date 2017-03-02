//
// Created by garrus on 21.02.17.
//

#include "StepsMediator.h"

const char request_topic[]     = "steps_controller/step_request";
const char response_topic[]    = "steps_controller/step_response";


StepsMediator::StepsMediator(ros::NodeHandle & nh, int maxBufferSize) :
        NodeMediatorBase(maxBufferSize)
{
    step_publisher = nh.advertise<geometry_msgs::PoseStamped>(request_topic, 100);
    step_subscriber = nh.subscribe(response_topic, 1, &StepsMediator::ResultCallback, this);

    //TODO: Временное решение
    DataSize = 7;
}

uint8_t StepsMediator::RequestLength()
{
    return 6;
}

bool StepsMediator::_SendRequest(const double* buffer, int count)
{
    if(count<RequestLength())
        return false;

    float xSl  = buffer[0], ySl  = buffer[1], zSl = buffer[2];
    float xSr  = buffer[3], ySr  = buffer[4], zSr = buffer[5];

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = xSl;
    pose.pose.position.y = ySl;
    pose.pose.position.y = zSl;

    //TODO: А вторая нога?

    ROS_INFO("steps_controller is called");
    step_publisher.publish(pose);

    return true;
}

//расчет шага закончен
void StepsMediator::ResultCallback(ar600_vision::StepResponse step)
{
    /* Вызываем функцию Done, чтобы указать, что расчет закончен.
     * В нее мы передаем лямбда-функцию, которая принимает
     * буфер и размер, в которые мы устанавливаем те данные,
     * которые хотим передать.
     *
     * Лямбда функция может захватывать значения из внешнего контекста,
     * в данном случае - str по ссылке.
     */

    Done([&step](double* buffer, int & count, int maxCount)
    {
        if(maxCount<7)
        {
            ROS_ERROR("StepsMediator: Inner buffer is too small to contains all result data");
            throw std::overflow_error("Inner buffer is too small to contains all result data");
        }


        buffer[0]=step.CanStep;
        buffer[1]=step.Pose.position.x;
        buffer[2]=step.Pose.position.y;
        buffer[3]=step.Pose.position.z;

        //TODO: А вторая нога?

        //count = 7;
    });
}


