//
// Created by user on 13.04.17.
//

#ifndef AR600_VISION_COORDCONVERTER_H
#define AR600_VISION_COORDCONVERTER_H

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include "tf/tf.h"
using namespace Eigen;

/*
 * Преобразования из координат в индексы на
 * proj_map и обратно
 */
class CoordConverter
{
public:

    static tf::Vector3 ToVector3(float x, float y);
    static tf::Vector3 ToVector3(Vector2f v);
    static tf::Vector3 ToVector3(Vector2i v);

    static geometry_msgs::Pose ToPose(tf::Vector3  v);

    static tf::Vector3 CoordToIndex(tf::Vector3 coord, nav_msgs::OccupancyGrid map);
    static tf::Vector3 IndexToCoord(tf::Vector3 index, nav_msgs::OccupancyGrid map);
};


#endif //AR600_VISION_COORDCONVERTER_H
