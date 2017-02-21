//
// Created by garrus on 21.02.17.
//

#include <ros/ros.h>

#ifndef AR600_VISION_STEPSMEDIATOR_H
#define AR600_VISION_STEPSMEDIATOR_H

#include "../NodeMediatorBase.h"
#include <std_msgs/String.h>


class StepsMediator : public NodeMediatorBase
{
public:

    /*!
     * Конструктор
     * @param nh дескриптор ноды ROS, чтобы подписвываться и публиковать
     * @param maxBufferSize размер буфера отпрвки, который будет выделен
     */
    StepsMediator(ros::NodeHandle & nh, int maxBufferSize);

    //Расчет шага закончен
    void ResultCallback(std_msgs::String str);

protected:

    /*!
     * Преобразует массив double в нужные параметры
     * и отправляет в топик
     * @param buffer
     * @param count
     */
    virtual void _SendRequest(const double* buffer, int count) override;

};


#endif //AR600_VISION_STEPSMEDIATOR_H
