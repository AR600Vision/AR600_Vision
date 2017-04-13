//
// Created by user on 13.04.17.
//

#include "ObstacleMediator.h"

ObstacleMediator::ObstacleMediator(ros::NodeHandle & nh):mutex()
{
    obstacle_subscriber = nh.subscribe("obstacle_detector/obstacle", 1, &ObstacleMediator::ObstacleCallback, this);
}

bool ObstacleMediator::IsObstacle()
{
    std::lock_guard<std::mutex> l(mutex);
    return isObstacle;
}

void ObstacleMediator::ObstacleCallback(ar600_vision::NearestObstacle obstacle)
{
    std::lock_guard<std::mutex> l(mutex);
    isObstacle = obstacle.IsObstacle;
}
