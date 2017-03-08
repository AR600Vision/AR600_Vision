//
// Created by garrus on 21.02.17.
//

#ifndef AR600_VISION_PATHMEDIATOR_H
#define AR600_VISION_PATHMEDIATOR_H

#include "../NodeMediatorBase.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>

class PathMediator : public NodeMediatorBase
{
public:

    /*!
     * Конструктор
     * @param nh дескриптор ноды ROS, чтобы подписвываться и публиковать
     * @param maxBufferSize размер буфера отпрвки, который будет выделен
     */
    PathMediator(ros::NodeHandle nh, int maxBufferSize);

    /*!
     * Отправляет запрос
     */
    void SendRequest(double posX, double posY);

    //Расчет тракетории законечен
    void Callback(nav_msgs::Path path);

private:
    ros::Subscriber path_subscriber;
    ros::Publisher goal_publisher;
};


#endif //AR600_VISION_PATHMEDIATOR_H
