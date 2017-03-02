//
// Created by garrus on 21.02.17.
//

#include "StepsMediator.h"

const char request_topic[]     = "steps_controller/step_request";
const char response_topic[]    = "steps_controller/step_response";


StepsMediator::StepsMediator(ros::NodeHandle & nh) :
        NodeMediatorBase(7)
{
    step_publisher = nh.advertise<geometry_msgs::PoseStamped>(request_topic, 100);
    step_subscriber = nh.subscribe(response_topic, 1, &StepsMediator::ResultCallback, this);
    stepCnt=0;
}

//Сколько данных требуется в запросе
uint8_t StepsMediator::RequestLength()
{
    return 6;
}

bool StepsMediator::_SendRequest(const double* buffer, int count)
{
    ROS_INFO("SendRequest. Count: %d", count);

    if(count<RequestLength())
        return false;

    //publish(buffer[0], buffer[1], buffer[2]);
    //_xs = buffer[3]; _ys=buffer[4]; _zs=buffer[5];

    //stepCnt++;

    Done([this, buffer](double* buffer1, int & count, int maxCount)
         {
             ROS_INFO("Done");

             if(maxCount<7)
             {
                 ROS_ERROR("StepsMediator: Inner buffer is too small to contains all result data");
                 throw std::overflow_error("Inner buffer is too small to contains all result data");
             }

             //Первая нога
             for(int i=0; i<6; i++)
                 buffer1[i]=buffer[i]+1;

             buffer1[6] = 1;

             count = 7;
         });

    return true;
}

//расчет шага закончен
void StepsMediator::ResultCallback(ar600_vision::StepResponse step)
{

    if(stepCnt==1)
    {
        //Посчитали первую, запускаем вторую
        publish(_xs, _ys, _zs);

        //Сохраняем пааметры левой ноги
        _xs = step.Pose.position.x;
        _ys = step.Pose.position.y;
        _zs = step.Pose.position.z;
        canStep = step.CanStep;

        stepCnt++;
        return;
    }


    /* Вызываем функцию Done, чтобы указать, что расчет закончен.
     * В нее мы передаем лямбда-функцию, которая принимает
     * буфер и размер, в которые мы устанавливаем те данные,
     * которые хотим передать.
     *
     * Лямбда функция может захватывать значения из внешнего контекста,
     * в данном случае - str по ссылке.
     */


    Done([&step, this](double* buffer, int & count, int maxCount)
    {
        if(maxCount<7)
        {
            ROS_ERROR("StepsMediator: Inner buffer is too small to contains all result data");
            throw std::overflow_error("Inner buffer is too small to contains all result data");
        }

        //Первая нога
        buffer[0]=step.Pose.position.x;
        buffer[1]=step.Pose.position.y;
        buffer[2]=step.Pose.position.z;

        //Вторая нога
        buffer[3]=_xs;
        buffer[4]=_ys;
        buffer[5]=_zs;
        buffer[6]=step.CanStep & canStep;

        count = 7;
    });

    stepCnt=0;
}

//Публикует в топик
void StepsMediator::publish(float x, float y, float z)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.y = z;

    ROS_INFO("steps_controller is called");
    step_publisher.publish(pose);
}


