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
    return 7;
}

bool StepsMediator::_SendRequest(const double* inBuffer, int count)
{
    if (count < RequestLength())
    {
        ROS_ERROR("StepsMediator: Not enough parameters");
        return false;
    }

    if (inBuffer[6] == 1)
    {
        publish(inBuffer[0], inBuffer[1], inBuffer[2]);
        _xs = inBuffer[3];
        _ys = inBuffer[4];
        _zs = inBuffer[5];

        stepCnt++;

        return false;
    }
    else
    {
        Done([&inBuffer, this](double* buffer, int & count, const int maxCount)
        {
            //memcpy(buffer, inBuffer, 7);
            for(int i = 0; i<7; i++)
                buffer[i]=inBuffer[i];
            count = 7;
        });

        return true;
    }

    //return true;
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


    Done([&step, this](double* buffer, int & count, const int maxCount)
    {
        if(maxCount<7)
        {
            ROS_ERROR("StepsMediator: Inner buffer is too small to contains all result data");
            throw std::overflow_error("Inner buffer is too small to contains all result data");
        }

        //Левая нога
        buffer[0]=_xs;
        buffer[1]=_ys;
        buffer[2]=_zs;

        //Правая нога
        buffer[3]=step.Pose.position.x;
        buffer[4]=step.Pose.position.y;
        buffer[5]=step.Pose.position.z;

        buffer[6]= (step.CanStep & canStep) ? 10 : 0;

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
    pose.pose.position.z = z;

    ROS_INFO("steps_controller is called");
    step_publisher.publish(pose);
}


