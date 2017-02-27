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
}

void StepsMediator::_SendRequest(const double* buffer, int count)
{
    if(count<8)
    {
        ROS_ERROR("StepsMediator: not enough arguments in request");
        throw std::invalid_argument("Not enough arguments in request");
    }

    float dX  = buffer[0], dY  = buffer[1],  dZ  = buffer[2];
    float dfX = buffer[3], dfY = buffer[4], dfZ = buffer[5];
    float xS  = buffer[6], yS  = buffer[7];

    //TODO: Опубилвковать смещения и повороты в TF

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = xS;
    pose.pose.position.y = yS;

    ROS_INFO("steps_controller is called");
    step_publisher.publish(pose);
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
        if(maxCount<10)
        {
            ROS_ERROR("StepsMediator: Inner buffer is too small to contains all result data");
            throw std::overflow_error("Inner buffer is too small to contains all result data");
        }

        //TODO: Запросить трансформацию и передать
        buffer[0]=0; buffer[1]=0; buffer[2]=0;
        buffer[3]=0; buffer[4]=0; buffer[5]=0;

        buffer[6]=step.CanStep;
        buffer[7]=step.Pose.position.x;
        buffer[8]=step.Pose.position.y;
        buffer[9]=step.Pose.position.z;

        count = 10;
    });
}


