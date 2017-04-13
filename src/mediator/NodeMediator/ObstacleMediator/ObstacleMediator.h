//
// Created by user on 13.04.17.
//

#ifndef AR600_VISION_OBSTACLEMEDIATOR_H
#define AR600_VISION_OBSTACLEMEDIATOR_H

#include <mutex>
#include <ros/ros.h>
#include "../../../../../../devel/include/ar600_vision/NearestObstacle.h"

/*
 * Подписывается на топик препятствий и
 * возващает есть\нет
 */
class ObstacleMediator
{
public:
    ObstacleMediator(ros::NodeHandle & nh);
    bool IsObstacle();

private:
    ros::Subscriber obstacle_subscriber;
    std::mutex mutex;
    bool isObstacle;

    void ObstacleCallback(ar600_vision::NearestObstacle obstacle);
};


#endif //AR600_VISION_OBSTACLEMEDIATOR_H
