//
// Created by garrus on 21.02.17.
//

#include "StepsMediator.h"

StepsMediator::StepsMediator(ros::NodeHandle & nh, int maxBufferSize) :
        NodeMediatorBase(maxBufferSize)
{
    ros::Publisher step_publisher = nh.advertise<std_msgs::String>("/steps_controller/calc_step_done", 100);
    ros::Subscriber step_subscriber = nh.subscribe("/steps_controller/calc_step", 1, &StepsMediator::ResultCallback, this);
}

void StepsMediator::_SendRequest(const double* buffer, int count)
{
    ROS_INFO("StepsMediator::SendRequest called");
    //TODO: отправить данные в топик
}

//расчет шага закончен
void StepsMediator::ResultCallback(std_msgs::String str)
{
    /* Вызываем функцию Done, чтобы указать, что расчет закончен.
     * В нее мы передаем лямбда-функцию, которая принимает
     * буфер и размер, в которые мы устанавливаем те данные,
     * которые хотим передать.
     *
     * Лямбда функция может захватывать значения из внешнего контекста,
     * в данном случае - str по ссылке.
     */

    Done([&str](double* buffer, int & count)
    {
        buffer[0]=0;
        count = 1;
    });
}


